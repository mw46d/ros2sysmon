"""ROS data models for ros collector."""

from dataclasses import dataclass
from datetime import datetime


@dataclass
class ROSNodeInfo:
    name: str
    pid: int
    cpu_percent: float
    memory_mb: float
    status: str  # "OK", "WARN", "ERROR"
    uptime: str
    namespace: str


@dataclass
class TopicMetrics:
    name: str
    frequency_hz: float
    target_frequency: float
    bandwidth_bps: int
    message_count: int
    msg_type: str
    status: str  # "OK", "SLOW", "IDLE", "ERROR"
    last_seen: datetime


@dataclass
class TFFrameInfo:
    frame_id: str
    parent_frame: str
    most_recent_transform: float  # timestamp
    oldest_transform: float  # timestamp
    is_root: bool  # True if this is a root frame
