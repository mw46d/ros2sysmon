"""Collection intervals configuration."""
from dataclasses import dataclass


@dataclass
class CollectionConfig:
    """Collection timing intervals configuration."""
    system_metrics: float
    network_ping: float
    ros_discovery: float
    hz_collection_duration: float