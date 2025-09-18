"""Topic configuration data structure."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class TopicConfig:
    """Configuration for a ROS topic with display and measurement options."""
    name: str
    target_frequency: Optional[float] = None
    measure_hz: bool = True
    display: bool = True