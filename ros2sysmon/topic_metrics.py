"""ROS2 topic metrics data structure."""
from dataclasses import dataclass
from datetime import datetime


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