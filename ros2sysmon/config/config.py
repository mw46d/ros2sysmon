"""Main configuration data structure."""
from dataclasses import dataclass
from .threshold_config import ThresholdConfig
from .ros_config import ROSConfig
from .display_config import DisplayConfig
from .collection_config import CollectionConfig


@dataclass
class Config:
    """Main configuration class."""
    refresh_rate: float
    max_alerts: int
    max_nodes_display: int
    max_topics_display: int
    collection_intervals: CollectionConfig
    thresholds: ThresholdConfig
    ros: ROSConfig
    display: DisplayConfig

    def __post_init__(self):
        """Fix invalid values."""
        if self.refresh_rate <= 0:
            self.refresh_rate = 15.0
        if self.max_alerts <= 0:
            self.max_alerts = 15
        if self.max_nodes_display <= 0:
            self.max_nodes_display = 15
        if self.max_topics_display <= 0:
            self.max_topics_display = 15