"""Main configuration data structure."""
from dataclasses import dataclass
from .threshold_config import ThresholdConfig
from .ros_config import ROSConfig
from .display_config import DisplayConfig


@dataclass
class Config:
    """Main configuration class."""
    refresh_rate: float
    max_alerts: int
    max_nodes_display: int
    max_topics_display: int
    thresholds: ThresholdConfig
    ros: ROSConfig
    display: DisplayConfig