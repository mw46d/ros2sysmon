"""Collection intervals configuration."""
from dataclasses import dataclass


@dataclass
class CollectionConfig:
    """Collection timing intervals configuration."""
    system_metrics: float
    network_ping: float
    ros_discovery: float
    hz_collection_duration: float

    def __post_init__(self):
        """Fix invalid values."""
        if self.system_metrics <= 0:
            self.system_metrics = 15.0
        if self.network_ping <= 0:
            self.network_ping = 15.0
        if self.ros_discovery <= 0:
            self.ros_discovery = 15.0
        if self.hz_collection_duration <= 0:
            self.hz_collection_duration = 15.0