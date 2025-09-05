"""System alert data structure."""
from dataclasses import dataclass
from datetime import datetime


@dataclass
class SystemAlert:
    """Alert messages for system monitoring."""
    level: str  # "INFO", "WARN", "ERROR"
    message: str
    timestamp: datetime
    category: str  # "SYSTEM", "ROS", "NETWORK"