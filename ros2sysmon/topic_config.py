"""Topic configuration data structure."""
from dataclasses import dataclass


@dataclass
class TopicConfig:
    """Configuration for a critical ROS topic."""
    name: str
    target_frequency: float