"""Alert data model for system monitoring."""
from dataclasses import dataclass
from datetime import datetime


@dataclass
class SystemAlert:
    level: str  # "I", "W", "E"
    message: str
    category: str  # "SYSTEM", "ROS", "NETWORK"