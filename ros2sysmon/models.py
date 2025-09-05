"""Data structures and classes for ros2top system monitor."""
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple


@dataclass
class SystemMetrics:
    """System-level metrics like CPU, memory, disk usage."""
    cpu_percent: float
    memory_percent: float
    disk_percent: float
    temperature: Optional[float]
    load_average: Tuple[float, float, float]  # 1min, 5min, 15min
    uptime: str
    battery_percent: Optional[float]
    network_latency: Optional[float]
    timestamp: datetime


@dataclass
class ROSNodeInfo:
    """Information about a running ROS2 node."""
    name: str
    pid: int
    cpu_percent: float
    memory_mb: float
    status: str  # "OK", "WARN", "ERROR"
    uptime: str
    namespace: str


@dataclass
class TopicMetrics:
    """Metrics for ROS2 topics like frequency and bandwidth."""
    name: str
    frequency_hz: float
    target_frequency: float
    bandwidth_bps: int
    message_count: int
    msg_type: str
    status: str  # "OK", "SLOW", "IDLE", "ERROR"
    last_seen: datetime


@dataclass
class SystemAlert:
    """Alert messages for system monitoring."""
    level: str  # "INFO", "WARN", "ERROR"
    message: str
    timestamp: datetime
    category: str  # "SYSTEM", "ROS", "NETWORK"