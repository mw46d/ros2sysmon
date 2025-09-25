"""Shared data store for thread-safe data access."""

import threading
from collections import deque
from typing import List, Optional

from ..collectors.ros_models import ROSNodeInfo, TFFrameInfo, TopicMetrics
from ..collectors.system_models import SystemMetrics
from .alerts import SystemAlert


class SharedDataStore:
    """Thread-safe data store for collectors to share data."""

    def __init__(self):
        """Initialize the shared data store with thread safety."""
        self._lock = threading.Lock()
        self.system_metrics: Optional[SystemMetrics] = None
        self.ros_nodes: list[ROSNodeInfo] = []
        self.topic_metrics: list[TopicMetrics] = []
        self.tf_frames: list[TFFrameInfo] = []
        self.processes: list[dict] = []
        self.alerts = deque(maxlen=50)

    def update_system_metrics(self, metrics: SystemMetrics):
        """Update system metrics in a thread-safe manner."""
        with self._lock:
            self.system_metrics = metrics

    def update_ros_nodes(self, nodes: List[ROSNodeInfo]):
        """Update ROS node list in a thread-safe manner."""
        with self._lock:
            self.ros_nodes = nodes.copy()

    def update_topic_metrics(self, topics: List[TopicMetrics]):
        """Update topic metrics in a thread-safe manner."""
        with self._lock:
            self.topic_metrics = topics.copy()

    def update_tf_frames(self, frames: List[TFFrameInfo]):
        """Update TF frame list in a thread-safe manner."""
        with self._lock:
            self.tf_frames = frames.copy()

    def update_processes(self, processes: List[dict]):
        """Update process list in a thread-safe manner."""
        with self._lock:
            self.processes = processes.copy()

    def add_alert(self, alert: SystemAlert):
        """Add an alert to the alert queue."""
        with self._lock:
            self.alerts.append(alert)

    def create_alert(self, level: str, message: str, category: str):
        """Create and add an alert in one step."""
        alert = SystemAlert(level, message, category)
        self.add_alert(alert)

    def get_system_data(self) -> tuple:
        """Get all system data in a thread-safe manner."""
        with self._lock:
            return (
                self.system_metrics,
                self.ros_nodes.copy(),
                self.topic_metrics.copy(),
                self.tf_frames.copy(),
                self.processes.copy(),
                list(self.alerts),
            )
