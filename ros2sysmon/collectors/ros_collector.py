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
from ..data_models import ROSNodeInfo, TopicMetrics, SystemAlert
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
        self.node = None  # Will be initialized in collect_loop
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        """Main collection loop - runs in background thread."""
        # Initialize ROS2 node in the collection thread
        try:
            self.node = Node('ros2sysmon_collector')
            # Setup topic subscribers on first run
            self._setup_topic_subscribers()
        except Exception:
            # ROS2 context not available - run without ROS features
            self.node = None
        
        while running.is_set():
            current_time = time.time()
            
            # Collect ROS data at specified interval
            if current_time - self.last_collection_time >= self.collection_interval:
                try:
                    if self.node:
                        # Spin ROS2 node to process callbacks
                        rclpy.spin_once(self.node, timeout_sec=0.01)
                    
                    nodes = self._discover_ros_nodes()
                    topics = self._collect_topic_metrics()
                    
                    shared_data.update_ros_nodes(nodes)
                    shared_data.update_topic_metrics(topics)
                    
                    # Generate ROS-related alerts
                    alerts = self._check_ros_thresholds(nodes, topics)
                    for alert in alerts:
                        shared_data.add_alert(alert)
                    
                    self.last_collection_time = current_time
                    
                except Exception:
                    # ROS2 not available or other issues - skip this cycle
                    pass
            else:
                # Still process ROS2 callbacks between collections
                try:
                    if self.node:
                        rclpy.spin_once(self.node, timeout_sec=0.01)
                except:
                    pass
            
            time.sleep(self.config.refresh_rate)
    
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
        """Discover running ROS2 nodes and their resource usage."""
        nodes = []
        
        try:
            # Get list of nodes using ros2 CLI
            result = subprocess.run(
                ['ros2', 'node', 'list'], 
                capture_output=True, 
                timeout=3,
                text=True
            )
            
            if result.returncode == 0:
                node_names = [name.strip() for name in result.stdout.strip().split('\n') if name.strip()]
                
                # Get process info for each node
                for node_name in node_names:
                    node_info = self._get_node_process_info(node_name)
                    if node_info:
                        nodes.append(node_info)
                        
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            # ROS2 not available or timeout
            pass
        
        return nodes
    
    def _get_node_process_info(self, node_name: str) -> Optional[ROSNodeInfo]:
        """Get process information for a ROS node."""
        try:
            # Find processes that might be this ROS node
            ros_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'cpu_percent', 'memory_info', 'create_time']):
                try:
                    cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                    if node_name.lstrip('/') in cmdline or 'ros2' in proc.info['name']:
                        ros_processes.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            if not ros_processes:
                return None
            
            # Use the first matching process (could be improved with better matching)
            proc = ros_processes[0]
            
            # Calculate uptime
            uptime_seconds = time.time() - proc.info['create_time']
            uptime_hours = int(uptime_seconds // 3600)
            uptime_minutes = int((uptime_seconds % 3600) // 60)
            uptime_str = f"{uptime_hours:02d}:{uptime_minutes:02d}"
            
            # Determine status based on CPU and memory usage
            cpu_percent = proc.info['cpu_percent']
            memory_mb = proc.info['memory_info'].rss / 1024 / 1024
            
            status = "OK"
            if cpu_percent > 50 or memory_mb > 500:
                status = "WARN"
            if cpu_percent > 80 or memory_mb > 1000:
                status = "ERROR"
            
            # Extract namespace from node name
            namespace = "/" if "/" not in node_name[1:] else node_name.rsplit("/", 1)[0]
            
            return ROSNodeInfo(
                name=node_name,
                pid=proc.info['pid'],
                cpu_percent=cpu_percent,
                memory_mb=memory_mb,
                status=status,
                uptime=uptime_str,
                namespace=namespace
            )
            
        except Exception:
            return None
    
    def _collect_topic_metrics(self) -> List[TopicMetrics]:
        """Collect metrics for ROS2 topics using direct subscriptions."""
        topics = []
        
        try:
            # Use our tracked topics from subscriptions
            for topic_name, topic_type in self.topic_types.items():
                frequency_hz = self._calculate_topic_hz(topic_name)
                message_count = self.message_counts.get(topic_name, 0)
                
                # Determine status based on frequency
                status = "OK"
                if frequency_hz == 0:
                    status = "IDLE"
                elif frequency_hz < 1.0:
                    status = "SLOW"
                
                # Estimate bandwidth (rough approximation)
                bandwidth_bps = int(frequency_hz * 100)  # Assume ~100 bytes per message
                
                # Set target frequency based on common topic patterns
                target_frequency = self._get_target_frequency(topic_name, topic_type)
                
                topic_metrics = TopicMetrics(
                    name=topic_name,
                    frequency_hz=frequency_hz,
                    target_frequency=target_frequency,
                    bandwidth_bps=bandwidth_bps,
                    message_count=message_count,
                    msg_type=topic_type,
                    status=status,
                    last_seen=datetime.now()
                )
                
                topics.append(topic_metrics)
                                
        except Exception:
            # ROS2 not available
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
        
        # Check node resource usage
        for node in nodes:
            if node.status == "ERROR":
                alerts.append(SystemAlert(
                    "ERROR",
                    f"ROS node {node.name} using high resources: {node.cpu_percent:.1f}% CPU, {node.memory_mb:.1f}MB",
                    datetime.now(),
                    "ROS"
                ))
            elif node.status == "WARN":
                alerts.append(SystemAlert(
                    "WARN",
                    f"ROS node {node.name} elevated usage: {node.cpu_percent:.1f}% CPU, {node.memory_mb:.1f}MB",
                    datetime.now(),
                    "ROS"
                ))
        
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