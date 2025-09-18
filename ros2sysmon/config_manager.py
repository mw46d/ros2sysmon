"""Configuration loading and management."""
import yaml
from .config import Config
from .threshold_config import ThresholdConfig
from .topic_config import TopicConfig
from .ros_config import ROSConfig
from .display_config import DisplayConfig
from .collection_config import CollectionConfig
from .panel_layout_config import PanelLayoutConfig


class ConfigManager:
    """Configuration loading and management."""
    
    @staticmethod
    def load_config(config_path: str) -> Config:
        """Load configuration from YAML file - let it crash if bad."""
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
        
        # Parse collection intervals
        collection_intervals = CollectionConfig(**config_data['collection_intervals'])
        
        # Parse thresholds
        thresholds = ThresholdConfig(**config_data['thresholds'])
        
        # Parse ROS config
        topics = [TopicConfig(**topic) for topic in config_data['ros']['config_topics']]
        ros_config = ROSConfig(
            config_topics=topics,
            node_patterns=config_data['ros']['node_patterns']
        )
        
        # Parse display config
        display_data = config_data['display']

        # Parse panel layout (with fallback to defaults)
        panel_layout_data = display_data.get('panel_layout', {})
        panel_layout = PanelLayoutConfig(**panel_layout_data)

        # Create display config
        display = DisplayConfig(
            show_colors=display_data['show_colors'],
            show_progress_bars=display_data['show_progress_bars'],
            time_format=display_data['time_format'],
            panel_layout=panel_layout
        )
        
        # Create main config
        return Config(
            refresh_rate=config_data['refresh_rate'],
            max_alerts=config_data['max_alerts'],
            max_nodes_display=config_data['max_nodes_display'],
            max_topics_display=config_data['max_topics_display'],
            collection_intervals=collection_intervals,
            thresholds=thresholds,
            ros=ros_config,
            display=display
        )