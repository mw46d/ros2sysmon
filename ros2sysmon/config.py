"""Configuration management for ros2top."""
import yaml
from dataclasses import dataclass
from typing import Dict, Any, List


@dataclass
class ThresholdConfig:
    """System threshold configuration for alerts."""
    cpu_warn: float
    cpu_error: float
    memory_warn: float
    memory_error: float
    disk_warn: float
    disk_error: float
    temperature_warn: float
    temperature_error: float
    network_latency_warn: int
    network_latency_error: int


@dataclass
class TopicConfig:
    """Configuration for a critical ROS topic."""
    name: str
    target_frequency: float


@dataclass
class ROSConfig:
    """ROS2 specific configuration."""
    critical_topics: List[TopicConfig]
    node_patterns: Dict[str, List[str]]


@dataclass
class DisplayConfig:
    """Display preferences configuration."""
    show_colors: bool
    show_progress_bars: bool
    time_format: str


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


class ConfigManager:
    """Configuration loading and management."""
    
    @staticmethod
    def load_config(config_path: str) -> Config:
        """Load configuration from YAML file - let it crash if bad."""
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
        
        # Parse thresholds
        thresholds = ThresholdConfig(**config_data['thresholds'])
        
        # Parse ROS config
        topics = [TopicConfig(**topic) for topic in config_data['ros']['critical_topics']]
        ros_config = ROSConfig(
            critical_topics=topics,
            node_patterns=config_data['ros']['node_patterns']
        )
        
        # Parse display config
        display = DisplayConfig(**config_data['display'])
        
        # Create main config
        return Config(
            refresh_rate=config_data['refresh_rate'],
            max_alerts=config_data['max_alerts'],
            max_nodes_display=config_data['max_nodes_display'],
            max_topics_display=config_data['max_topics_display'],
            thresholds=thresholds,
            ros=ros_config,
            display=display
        )