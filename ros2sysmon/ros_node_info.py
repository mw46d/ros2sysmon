"""ROS2 node information data structure."""
from dataclasses import dataclass


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