"""ROS2 configuration data structure."""
from dataclasses import dataclass
from typing import Dict, List
from .topic_config import TopicConfig


@dataclass
class ROSConfig:
    """ROS2 specific configuration."""
    critical_topics: List[TopicConfig]
    node_patterns: Dict[str, List[str]]