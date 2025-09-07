"""ROS2 node and topic collector for monitoring ROS2 system."""
import time
import threading
import subprocess
import json
import psutil
from datetime import datetime
from typing import List, Optional, Dict
from collections import defaultdict, deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from tf2_ros import TransformListener, Buffer
import yaml
from ..data_models import ROSNodeInfo, TopicMetrics, SystemAlert, TFFrameInfo
from ..shared_data import SharedDataStore
from ..config import Config


class ROSCollector:
    """Collects ROS2 node and topic metrics using direct ROS2 subscriptions."""
    
    def __init__(self, config: Config):
        """Initialize the ROS collector."""
        self.config = config
        self.last_collection_time = 0
        self.collection_interval = 2.0  # Collect every 2 seconds
        self.window_size = 50  # Number of messages to track for Hz calculation
        
        # Topic frequency tracking
        self.timestamps = defaultdict(lambda: deque(maxlen=self.window_size))
        self.message_counts = defaultdict(int)
        self.topic_types = {}
        self.subscribers = {}
        
        # TF monitoring
        self.tf_buffer = None
        self.tf_listener = None
        
        self.node = None  # Will be initialized in collect_loop
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        """Main collection loop - runs in background thread."""
        # Initialize ROS2 node in the collection thread
        try:
            self.node = Node('ros2sysmon_collector')
            # print("ROS Collector: Successfully created ROS2 node")
            # Setup TF listener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            # print("ROS Collector: Successfully created TF listener")
            # Setup topic subscribers on first run
            self._setup_topic_subscribers()
            # print("ROS Collector: Successfully setup topic subscribers")
        except Exception as e:
            # ROS2 context not available - run without ROS features
            # print(f"ROS Collector: Failed to initialize ROS2 node: {e}")
            self.node = None
            self.tf_buffer = None
            self.tf_listener = None
        
        # Get collection intervals
        ros_interval = self.config.collection_intervals.ros_discovery
        callback_interval = self.config.collection_intervals.ros_callbacks
        
        # Initial collection
        self._do_ros_collection(shared_data)
        
        if ros_interval == 0.0:
            # Zero interval: run once and wait for manual refresh
            while running.is_set():
                # Still process ROS2 callbacks if configured
                if callback_interval > 0.0 and self.node:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=callback_interval)
                    except Exception:
                        pass
                
                if self.manual_refresh_requested:
                    self.manual_refresh_requested = False
                    self._do_ros_collection(shared_data)
                
                time.sleep(0.1)  # Small sleep to avoid busy waiting
        else:
            # Normal interval-based collection
            last_collection_time = time.time()
            while running.is_set():
                current_time = time.time()
                
                # Check if it's time for ROS collection
                if current_time - last_collection_time >= ros_interval:
                    self._do_ros_collection(shared_data)
                    last_collection_time = current_time
                
                # Process ROS2 callbacks if configured
                if callback_interval > 0.0 and self.node:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=callback_interval)
                    except Exception:
                        pass
                
                time.sleep(0.01)  # Small delay to prevent busy waiting
    
    def _do_ros_collection(self, shared_data: SharedDataStore):
        """Perform one ROS collection cycle."""
        try:
            nodes = self._discover_ros_nodes()
            topics = self._collect_topic_metrics()
            tf_frames = self._get_tf_frames()
            
            # Debug output (uncomment for troubleshooting)
            # print(f"ROS Collector: Found {len(nodes)} nodes, {len(topics)} topics, {len(tf_frames)} TF frames")
            
            shared_data.update_ros_nodes(nodes)
            shared_data.update_topic_metrics(topics)
            shared_data.update_tf_frames(tf_frames)
            
            # Generate ROS-related alerts
            alerts = self._check_ros_thresholds(nodes, topics)
            for alert in alerts:
                shared_data.add_alert(alert)
            
        except Exception as e:
            # ROS2 not available or other issues - clear data
            shared_data.update_ros_nodes([])
            shared_data.update_topic_metrics([])
            shared_data.update_tf_frames([])
            # print(f"ROS collection failed: {e}")  # Debug output
    
    def trigger_manual_refresh(self):
        """Trigger an immediate collection cycle."""
        self.manual_refresh_requested = True
    
    def _setup_topic_subscribers(self):
        """Setup subscribers for all available topics to track frequency."""
        if not self.node:
            return
            
        try:
            # Get all topics and their types
            topic_names_and_types = self.node.get_topic_names_and_types()
            
            for topic_name, topic_types in topic_names_and_types:
                if topic_name not in self.subscribers and topic_types:
                    try:
                        # Use the first available message type
                        topic_type = topic_types[0]
                        self.topic_types[topic_name] = topic_type
                        
                        # Import and create subscriber
                        msg_class = get_message(topic_type)
                        subscriber = self.node.create_subscription(
                            msg_class,
                            topic_name,
                            lambda msg, t=topic_name: self._topic_callback(t),
                            qos_profile=qos_profile_sensor_data
                        )
                        
                        self.subscribers[topic_name] = subscriber
                        
                    except Exception:
                        # Skip topics we can't subscribe to
                        pass
                        
        except Exception:
            # ROS2 not available
            pass
    
    def _topic_callback(self, topic_name: str):
        """Callback for topic messages to track frequency."""
        current_time = time.time()
        self.timestamps[topic_name].append(current_time)
        self.message_counts[topic_name] += 1
    
    def _calculate_topic_hz(self, topic_name: str) -> float:
        """Calculate Hz for a specific topic based on message timestamps."""
        if topic_name not in self.timestamps or len(self.timestamps[topic_name]) < 2:
            return 0.0
            
        timestamps = list(self.timestamps[topic_name])
        if len(timestamps) < 2:
            return 0.0
            
        # Calculate time span
        time_span = timestamps[-1] - timestamps[0]
        if time_span == 0:
            return 0.0
            
        # Calculate frequency
        num_messages = len(timestamps) - 1
        hz = num_messages / time_span
        
        return hz
    
    def _discover_ros_nodes(self) -> List[ROSNodeInfo]:
        """Discover running ROS2 nodes by name only."""
        nodes = []
        
        try:
            # Get list of nodes using ros2 CLI with longer timeout
            result = subprocess.run(
                ['ros2', 'node', 'list'], 
                capture_output=True, 
                timeout=10,  # Increased from 3 to 10 seconds
                text=True
            )
            
            if result.returncode == 0:
                node_names = [name.strip() for name in result.stdout.strip().split('\n') if name.strip()]
                
                # Create simple node info with just names
                for node_name in node_names:
                    node_info = ROSNodeInfo(
                        name=node_name,
                        pid=0,  # Not tracking process info
                        cpu_percent=0.0,  # Not tracking CPU
                        memory_mb=0.0,  # Not tracking memory
                        status="ACTIVE",  # Simple active status
                        uptime="--:--",  # Not tracking uptime
                        namespace="/" if "/" not in node_name[1:] else node_name.rsplit("/", 1)[0]
                    )
                    nodes.append(node_info)
            # else:
            #     print(f"ros2 node list failed with return code: {result.returncode}")
            #     print(f"stderr: {result.stderr}")
                        
        except subprocess.TimeoutExpired:
            # print("ros2 node list timed out after 10 seconds")
            pass
        except FileNotFoundError:
            # print("ros2 command not found")
            pass
        except Exception as e:
            # print(f"Unexpected error running ros2 node list: {e}")
            pass
        
        return nodes
    
    
    def _collect_topic_metrics(self) -> List[TopicMetrics]:
        """Collect metrics for ROS2 topics using CLI-based discovery."""
        topics = []
        
        try:
            # Get list of topics using ros2 CLI
            result = subprocess.run(
                ['ros2', 'topic', 'list', '-t'], 
                capture_output=True, 
                timeout=5,
                text=True
            )
            
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if line.strip() and ' ' in line:
                        # Parse "topic_name message_type" format
                        parts = line.strip().split()
                        if len(parts) >= 2:
                            topic_name = parts[0]
                            msg_type = parts[-1]  # Take the last part as message type
                            
                            # Calculate frequency if we have subscription data
                            frequency_hz = self._calculate_topic_hz(topic_name) if topic_name in self.timestamps else 0.0
                            
                            # Determine status based on frequency
                            status = "OK"
                            if frequency_hz == 0:
                                status = "IDLE"
                            elif frequency_hz < 1.0:
                                status = "SLOW"
                            
                            # Set target frequency based on common topic patterns
                            target_frequency = self._get_target_frequency(topic_name, msg_type)
                            
                            # Estimate bandwidth (rough approximation)
                            bandwidth_bps = int(frequency_hz * 100)  # Assume ~100 bytes per message
                            
                            # Get message count
                            message_count = self.message_counts.get(topic_name, 0)
                            
                            topic_metrics = TopicMetrics(
                                name=topic_name,
                                frequency_hz=frequency_hz,
                                target_frequency=target_frequency,
                                bandwidth_bps=bandwidth_bps,
                                message_count=message_count,
                                msg_type=msg_type,
                                status=status,
                                last_seen=datetime.now()
                            )
                            topics.append(topic_metrics)
            # else:
            #     print(f"ros2 topic list failed with return code: {result.returncode}")
                
        except Exception as e:
            # Return empty list on error
            # print(f"Topic collection failed: {e}")  # Uncomment for debugging
            pass
            
        return topics
    
    def _get_target_frequency(self, topic_name: str, msg_type: str) -> float:
        """Get expected target frequency for a topic based on common patterns."""
        # Common frequency expectations for different topic types
        if '/cmd_vel' in topic_name or '/twist' in topic_name:
            return 10.0  # Control topics usually 10Hz
        elif '/odom' in topic_name:
            return 30.0  # Odometry usually 30Hz
        elif '/scan' in topic_name or 'lidar' in topic_name:
            return 10.0  # LiDAR scans usually 10Hz
        elif '/image' in topic_name or '/camera' in topic_name:
            return 30.0  # Camera topics usually 30Hz
        elif '/imu' in topic_name:
            return 100.0  # IMU usually high frequency
        elif '/tf' in topic_name:
            return 100.0  # Transform data high frequency
        else:
            return 1.0  # Default expectation
    
    def _check_ros_thresholds(self, nodes: List[ROSNodeInfo], topics: List[TopicMetrics]) -> List[SystemAlert]:
        """Check ROS nodes and topics for potential issues."""
        alerts = []
        
        # No longer checking node resource usage since we don't track it
        
        # Check topic health
        for topic in topics:
            if topic.status == "IDLE":
                alerts.append(SystemAlert(
                    "WARN",
                    f"Topic {topic.name} appears idle (0 Hz)",
                    datetime.now(),
                    "ROS"
                ))
            elif topic.status == "SLOW" and topic.target_frequency > 5.0:
                alerts.append(SystemAlert(
                    "WARN",
                    f"Topic {topic.name} slow: {topic.frequency_hz:.1f}Hz (expected {topic.target_frequency:.1f}Hz)",
                    datetime.now(),
                    "ROS"
                ))
        
        return alerts
    
    def _get_tf_frames(self) -> List[TFFrameInfo]:
        """Get current TF frame information."""
        if not self.tf_buffer:
            return []
            
        try:
            # Get all frames as YAML
            all_frames_yaml = self.tf_buffer.all_frames_as_yaml()
            
            # Parse YAML to get frame names and relationships
            frames_dict = yaml.safe_load(all_frames_yaml)
            
            if not frames_dict:
                return []
            
            # Find root frames (frames that appear as parents but don't exist as actual frames)
            all_frame_names = set(frames_dict.keys())
            all_parents = set()
            
            for frame_info in frames_dict.values():
                parent = frame_info.get('parent')
                if parent and parent != '':
                    all_parents.add(parent)
            
            # Root frames are parents that don't exist as actual frames
            root_frame_names = all_parents - all_frame_names
            
            tf_frames = []
            
            # Add root frames (these won't have frame_info, so create dummy entries)
            for root_name in sorted(root_frame_names):
                tf_frames.append(TFFrameInfo(
                    frame_id=root_name,
                    parent_frame='ROOT',  # Root frames have no parent
                    most_recent_transform=0.0,
                    oldest_transform=0.0,
                    is_root=True
                ))
            
            # Add all actual frames as child frames
            for frame_id, frame_info in sorted(frames_dict.items()):
                tf_frames.append(TFFrameInfo(
                    frame_id=frame_id,
                    parent_frame=frame_info.get('parent', ''),
                    most_recent_transform=frame_info.get('most_recent_transform', 0.0),
                    oldest_transform=frame_info.get('oldest_transform', 0.0),
                    is_root=False
                ))
            
            return tf_frames
            
        except Exception as e:
            # Return empty list on error
            return []