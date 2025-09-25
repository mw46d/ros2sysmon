"""Main entry point for ros2sysmon system monitor."""
import argparse
import time
import os
import threading
import rclpy
from ament_index_python.packages import get_package_share_directory
from .config.config_manager import ConfigManager
from .core.data_manager import DataCollectionManager
from .ui.display_manager import DisplayManager


def main():
    """Main entry point for the application."""
    parser = argparse.ArgumentParser(description="ROS2 System Monitor")
    package_share = get_package_share_directory('ros2sysmon')
    default_config = os.path.join(package_share, 'config', 'default_config.yaml')
    
    parser.add_argument("--config", default=default_config)
    parser.add_argument("--refresh-rate", type=float, default=5.0)
    parser.add_argument("--no-color", action="store_true")
    args = parser.parse_args()
    
    
    # Initialize ROS2 
    rclpy.init()
    
    # Load configuration - let it crash if bad
    config = ConfigManager.load_config(args.config)
    config.refresh_rate = args.refresh_rate
    
    # Initialize data collection and display managers
    data_manager = DataCollectionManager(config)
    display_manager = DisplayManager(config)
    shared_data = data_manager.get_shared_data()
    
    # Start data collection
    data_manager.start_collection()
    
    try:
        # Run Rich display (blocking)
        display_manager.run_display(shared_data, data_manager)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        data_manager.stop_collection()
        rclpy.shutdown()


if __name__ == "__main__":
    main()