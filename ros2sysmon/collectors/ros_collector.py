"""ROS2 node and topic collector for monitoring ROS2 system."""

import subprocess
import threading
import time
from collections import defaultdict
from datetime import datetime
from typing import List

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener

from ..config.config import Config
from ..core.alerts import SystemAlert
from ..core.shared_data import SharedDataStore
from .ros_models import ROSNodeInfo, TFFrameInfo, TopicMetrics


class ROSCollector:
    def __init__(self, config: Config):
        self.config = config
        self.last_collection_time = 0
        self.manual_refresh_requested = False

        # Windowed Hz collection approach
        self.collection_window_timestamps = defaultdict(list)
        self.collection_window_start_time = None
        self.collection_active = False

        # Topic subscription tracking
        self.topic_types = {}
        self.subscribers = {}

        # TF monitoring
        self.tf_buffer = None
        self.tf_listener = None

        self.node = None  # Will be initialized in collect_loop

    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        self._initialize_ros_node(shared_data)

        ros_interval = self.config.collection_intervals.ros_discovery
        hz_duration = self.config.collection_intervals.hz_collection_duration

        self._do_ros_discovery_only(shared_data)
        shared_data.create_alert(
            "I", "ROS collector ready - press 'r' to refresh", "ROS"
        )
        self._run_collection_loop(shared_data, running, ros_interval, hz_duration)

    def _initialize_ros_node(self, shared_data):
        try:
            self.node = Node("ros2sysmon_collector")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            self._setup_topic_subscribers(shared_data)

            shared_data.create_alert(
                "I",
                "ROS Collector: Successfully setup topic subscribers",
                "ros_collector",
            )
        except (ImportError, RuntimeError) as e:
            shared_data.create_alert(
                "E", f"Failed to initialize ROS node: {e}", "ros_collector"
            )
            self.node = None
            self.tf_buffer = None
            self.tf_listener = None

    def _run_collection_loop(self, shared_data, running, ros_interval, hz_duration):
        if ros_interval == 0.0:
            self._run_manual_refresh_loop(shared_data, running, hz_duration)
        else:
            self._run_interval_based_loop(
                shared_data, running, ros_interval, hz_duration
            )

    def _run_manual_refresh_loop(self, shared_data, running, hz_duration):
        while running.is_set():
            if self.manual_refresh_requested:
                self.manual_refresh_requested = False
                self._do_windowed_collection(shared_data, hz_duration)
            time.sleep(0.1)

    def _run_interval_based_loop(self, shared_data, running, ros_interval, hz_duration):
        last_collection_time = time.time()
        while running.is_set():
            current_time = time.time()
            if current_time - last_collection_time >= ros_interval:
                self._do_windowed_collection(shared_data, hz_duration)
                last_collection_time = current_time
            time.sleep(0.01)

    def _do_windowed_collection(self, shared_data: SharedDataStore, hz_duration: float):
        """Perform windowed Hz collection with start/stop phases."""
        try:
            # 1. Start collection window
            self.start_collection_window(shared_data)

            # 2. Collect for the specified duration while spinning ROS
            end_time = time.time() + hz_duration
            while time.time() < end_time:
                if self.node:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.01)
                    except (RuntimeError, OSError) as e:
                        shared_data.create_alert("E", f"ROS spin error: {e}", "ROS")
                time.sleep(0.001)  # Small sleep to prevent busy waiting

            # 3. Stop collection window
            self.stop_collection_window(shared_data)

            # 4. Collect other ROS data and calculate Hz
            nodes = self._discover_ros_nodes(shared_data)
            topics = self._collect_topic_metrics(shared_data)
            tf_frames = self._get_tf_frames()

            shared_data.update_ros_nodes(nodes)
            shared_data.update_topic_metrics(topics)
            shared_data.update_tf_frames(tf_frames)

            # Generate ROS-related alerts
            alerts = self._check_ros_thresholds(nodes, topics)
            for alert in alerts:
                shared_data.add_alert(alert)

        except (RuntimeError, OSError, ImportError) as e:
            shared_data.create_alert("E", f"Windowed collection failed: {e}", "ROS")
            shared_data.update_ros_nodes([])
            shared_data.update_topic_metrics([])
            shared_data.update_tf_frames([])

    def _do_ros_discovery_only(self, shared_data: SharedDataStore):
        """Perform ROS discovery without Hz collection (for initial setup)."""
        try:
            nodes = self._discover_ros_nodes(shared_data)
            tf_frames = self._get_tf_frames()

            # Create empty topic metrics for initial display
            topics = []

            shared_data.update_ros_nodes(nodes)
            shared_data.update_topic_metrics(topics)
            shared_data.update_tf_frames(tf_frames)

        except (RuntimeError, OSError, ImportError) as e:
            shared_data.create_alert("E", f"ROS discovery failed: {e}", "ROS")
            shared_data.update_ros_nodes([])
            shared_data.update_topic_metrics([])
            shared_data.update_tf_frames([])

    def _do_ros_collection(self, shared_data: SharedDataStore):
        """Perform one ROS collection cycle."""
        try:
            nodes = self._discover_ros_nodes(shared_data)
            topics = self._collect_topic_metrics(shared_data)
            tf_frames = self._get_tf_frames()

            # Debug output (uncomment for troubleshooting)

            shared_data.update_ros_nodes(nodes)
            shared_data.update_topic_metrics(topics)
            shared_data.update_tf_frames(tf_frames)

            # Generate ROS-related alerts
            alerts = self._check_ros_thresholds(nodes, topics)
            for alert in alerts:
                shared_data.add_alert(alert)

        except (RuntimeError, OSError, ImportError) as e:
            shared_data.create_alert("E", f"ROS collection failed: {e}", "ROS")
            shared_data.update_ros_nodes([])
            shared_data.update_topic_metrics([])
            shared_data.update_tf_frames([])

    def trigger_manual_refresh(self):
        """Trigger an immediate collection cycle."""
        self.manual_refresh_requested = True

    def start_collection_window(self, shared_data: SharedDataStore):
        """Start a new Hz collection window."""
        self.collection_window_start_time = time.time()
        self.collection_active = True
        # Clear previous window data
        self.collection_window_timestamps.clear()

        # Add alert message
        shared_data.create_alert("I", "Starting ROS data collection...", "ROS")

    def stop_collection_window(self, shared_data: SharedDataStore):
        """Stop the current Hz collection window."""
        self.collection_active = False
        duration = self.config.collection_intervals.hz_collection_duration
        self.collection_window_start_time = None

        # Add alert message
        shared_data.create_alert(
            "I", f"ROS data collection complete ({duration}s)", "ROS"
        )

    def _setup_topic_subscribers(self, shared_data):
        # Get configured topics (excluding wildcard)
        config_topics = [topic for topic in self.config.ros.config_topics if topic.name != "*"]

        # Only subscribe to topics that are explicitly configured and have measure_hz=True
        configured_hz_topics = [topic for topic in config_topics if topic.measure_hz]

        shared_data.create_alert("D", f"Config Hz topics: {[t.name for t in configured_hz_topics]}", "ros_collector")

        for topic_config in configured_hz_topics:
            topic_name = topic_config.name

            if topic_name not in self.subscribers:
                try:
                    # Get topic info from CLI to find the message type
                    result = self._run_topic_list_command()
                    if result and result.returncode == 0:
                        # Find this specific topic in the CLI output
                        lines = result.stdout.strip().split("\n")
                        topic_type = None
                        for line in lines:
                            if line.strip().startswith(topic_name + " "):
                                parts = line.strip().split()
                                if len(parts) >= 2:
                                    topic_type = parts[-1].strip("[](){}")
                                    break

                        if topic_type:
                            self.topic_types[topic_name] = topic_type

                            # Import and create subscriber
                            msg_class = get_message(topic_type)

                            # Create callback function that properly captures topic_name
                            def make_callback(topic):
                                return lambda msg: self._topic_callback(topic)

                            subscriber = self.node.create_subscription(
                                msg_class,
                                topic_name,
                                make_callback(topic_name),
                                qos_profile=qos_profile_sensor_data,
                            )

                            self.subscribers[topic_name] = subscriber

                            # Debug alert for successful subscription
                            shared_data.create_alert(
                                "I",
                                f"Subscribed to {topic_name} (type: {topic_type})",
                                "ros_collector",
                            )
                        else:
                            shared_data.create_alert(
                                "W",
                                f"Topic {topic_name} not found in ROS system",
                                "ros_collector",
                            )
                except Exception as e:
                    shared_data.create_alert(
                        "E",
                        f"Failed to subscribe to {topic_name}: {e}",
                        "ros_collector",
                    )

    def _topic_callback(self, topic_name: str):
        """Callback for topic messages to track frequency."""
        current_time = time.time()

        # Only collect timestamps during active collection windows
        if self.collection_active:
            self.collection_window_timestamps[topic_name].append(current_time)

    def _get_topics_for_hz_measurement(self, shared_data):
        """Get configuration about which topics should have Hz measured."""
        config_topics = [topic for topic in self.config.ros.config_topics if topic.name != "*"]

        # Create lookup dictionaries (no wildcards)
        configured_topic_names = {topic.name for topic in config_topics}
        measure_hz_settings = {topic.name: topic.measure_hz for topic in config_topics}

        # Debug alert showing configuration
        measure_hz_topics = [name for name, measure in measure_hz_settings.items() if measure]
        shared_data.create_alert(
            "D",
            f"Config topics: {list(configured_topic_names)}, measure_hz: {measure_hz_topics}",
            "ros_collector",
        )

        return {
            "configured_topic_names": configured_topic_names,
            "measure_hz_settings": measure_hz_settings,
        }

    def _should_measure_hz(self, topic_name: str, topics_to_measure: dict) -> bool:
        """Determine if Hz should be measured for a specific topic."""
        configured_topic_names = topics_to_measure["configured_topic_names"]
        measure_hz_settings = topics_to_measure["measure_hz_settings"]

        if topic_name in configured_topic_names:
            # Topic is explicitly configured
            return measure_hz_settings.get(topic_name, True)
        # Topic not configured, don't measure
        return False

    def _calculate_topic_hz(self, topic_name: str) -> float:
        """Calculate Hz for a specific topic based on windowed collection."""
        if topic_name not in self.collection_window_timestamps:
            return 0.0

        timestamps = self.collection_window_timestamps[topic_name]
        if len(timestamps) == 0:
            return 0.0

        # Calculate frequency: messages per second during the window
        window_duration = self.config.collection_intervals.hz_collection_duration
        message_count = len(timestamps)
        hz = message_count / window_duration

        return hz

    def _discover_ros_nodes(self, shared_data) -> List[ROSNodeInfo]:
        result = self._run_node_list_command(shared_data)

        if result is None:
            return []

        if result.returncode == 0:
            return self._parse_node_list_output(result.stdout)
        else:
            shared_data.create_alert(
                "E",
                f"ros2 node list failed with return code: {result.returncode}, stderr: {result.stderr}",
                "ros_collector",
            )
            return []

    def _run_node_list_command(self, shared_data):
        try:
            return subprocess.run(
                ["ros2", "node", "list"],
                check=False,
                capture_output=True,
                timeout=10,
                text=True,
            )
        except subprocess.TimeoutExpired:
            self._handle_node_list_timeout(shared_data)
            return None
        except FileNotFoundError:
            self._handle_ros_command_not_found(shared_data)
            return None
        except (OSError, subprocess.SubprocessError):
            return None

    def _parse_node_list_output(self, stdout):
        node_names = [
            name.strip() for name in stdout.strip().split("\n") if name.strip()
        ]

        return [self._create_node_info(node_name) for node_name in node_names]

    def _create_node_info(self, node_name):
        return ROSNodeInfo(
            name=node_name,
            pid=0,
            cpu_percent=0.0,
            memory_mb=0.0,
            status="ACTIVE",
            uptime="--:--",
            namespace="/" if "/" not in node_name[1:] else node_name.rsplit("/", 1)[0],
        )

    def _handle_node_list_timeout(self, shared_data):
        shared_data.create_alert(
            "W", "ros2 node list timed out after 10 seconds", "ros_collector"
        )

    def _handle_ros_command_not_found(self, shared_data):
        shared_data.create_alert("E", "ros2 command not found", "ros_collector")

    def _collect_topic_metrics(self, shared_data) -> List[TopicMetrics]:
        topics = []
        result = self._run_topic_list_command()

        if result is None:
            return topics

        if result.returncode == 0:
            lines = result.stdout.strip().split("\n")
            for line in lines:
                topic_data = self._parse_topic_line(line)
                if topic_data:
                    topic_metrics = self._create_topic_metrics(*topic_data)
                    topics.append(topic_metrics)
        else:
            self._handle_topic_list_error(shared_data, result.returncode)

        return topics

    def _run_topic_list_command(self):
        try:
            return subprocess.run(
                ["ros2", "topic", "list", "-t"],
                check=False,
                capture_output=True,
                timeout=5,
                text=True,
            )
        except (OSError, subprocess.SubprocessError):
            return None

    def _parse_topic_line(self, line):
        if not (line.strip() and " " in line):
            return None

        parts = line.strip().split()
        if len(parts) < 2:
            return None

        topic_name = parts[0]
        msg_type = parts[-1].strip("[](){}")
        return topic_name, msg_type

    def _create_topic_metrics(self, topic_name, msg_type):
        frequency_hz = (
            self._calculate_topic_hz(topic_name)
            if topic_name in self.collection_window_timestamps
            else 0.0
        )

        status = self._determine_topic_status(frequency_hz)
        target_frequency = self._get_target_frequency(topic_name, msg_type)
        bandwidth_bps = int(frequency_hz * 100)

        return TopicMetrics(
            name=topic_name,
            frequency_hz=frequency_hz,
            target_frequency=target_frequency,
            bandwidth_bps=bandwidth_bps,
            message_count=0,
            msg_type=msg_type,
            status=status,
            last_seen=datetime.now(),
        )

    def _determine_topic_status(self, frequency_hz):
        if frequency_hz == 0:
            return "IDLE"
        elif frequency_hz < 1.0:
            return "SLOW"
        return "OK"

    def _handle_topic_list_error(self, shared_data, return_code):
        shared_data.create_alert(
            "E",
            f"ros2 topic list failed with return code: {return_code}",
            "ros_collector",
        )

    def _get_target_frequency(self, topic_name: str, msg_type: str) -> float:
        config_topics = self.config.ros.config_topics

        # Check for exact topic name match first
        for topic in config_topics:
            if topic.name == topic_name and topic.target_frequency is not None:
                return topic.target_frequency

        # Check wildcard config
        wildcard_config = next(
            (topic for topic in config_topics if topic.name == "*"), None
        )
        if wildcard_config and wildcard_config.target_frequency is not None:
            return wildcard_config.target_frequency

        return 0.0

    def _check_ros_thresholds(
        self, nodes: List[ROSNodeInfo], topics: List[TopicMetrics]
    ) -> List[SystemAlert]:
        """Check ROS nodes and topics for potential issues."""
        alerts = []

        # No longer checking node resource usage since we don't track it

        # Check topic health
        for topic in topics:
            if topic.status == "IDLE":
                alerts.append(
                    SystemAlert(
                        "W",
                        f"Topic {topic.name} appears idle (0 Hz)",
                        "ROS",
                    )
                )
            elif topic.status == "SLOW" and topic.target_frequency > 5.0:
                alerts.append(
                    SystemAlert(
                        "W",
                        f"Topic {topic.name} slow: {topic.frequency_hz:.1f}Hz (expected {topic.target_frequency:.1f}Hz)",
                        "ROS",
                    )
                )

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
                parent = frame_info.get("parent")
                if parent and parent != "":
                    all_parents.add(parent)

            # Root frames are parents that don't exist as actual frames
            root_frame_names = all_parents - all_frame_names

            tf_frames = []

            # Add root frames (these won't have frame_info, so create dummy entries)
            for root_name in sorted(root_frame_names):
                tf_frames.append(
                    TFFrameInfo(
                        frame_id=root_name,
                        parent_frame="ROOT",  # Root frames have no parent
                        most_recent_transform=0.0,
                        oldest_transform=0.0,
                        is_root=True,
                    )
                )

            # Add all actual frames as child frames
            for frame_id, frame_info in sorted(frames_dict.items()):
                tf_frames.append(
                    TFFrameInfo(
                        frame_id=frame_id,
                        parent_frame=frame_info.get("parent", ""),
                        most_recent_transform=frame_info.get(
                            "most_recent_transform", 0.0
                        ),
                        oldest_transform=frame_info.get("oldest_transform", 0.0),
                        is_root=False,
                    )
                )

            return tf_frames

        except (RuntimeError, OSError, ImportError):
            return []
