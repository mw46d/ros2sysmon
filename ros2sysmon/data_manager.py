"""Data collection management for coordinating multiple collectors."""
import threading
from typing import List
from .shared_data import SharedDataStore
from .collectors.system_collector import SystemCollector
from .collectors.network_collector import NetworkCollector
from .collectors.ros_collector import ROSCollector
from .config import Config


class DataCollectionManager:
    """Manages multiple data collectors with coordinated threading."""
    
    def __init__(self, config: Config):
        """Initialize collection manager with collectors."""
        self.config = config
        self.collectors = []
        self.threads = []
        self.running = threading.Event()
        self.shared_data = SharedDataStore()
        
        # Initialize collectors
        self.collectors = [
            SystemCollector(config),
            NetworkCollector(config),
            ROSCollector(config)
        ]
    
    def start_collection(self):
        """Start all data collection threads."""
        self.running.set()
        
        for collector in self.collectors:
            thread = threading.Thread(
                target=collector.collect_loop,
                args=(self.shared_data, self.running),
                daemon=True
            )
            thread.start()
            self.threads.append(thread)
    
    def stop_collection(self):
        """Stop all data collection threads."""
        self.running.clear()
        
        # Wait for threads to finish
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=1)
    
    def get_shared_data(self) -> SharedDataStore:
        """Get the shared data store for reading."""
        return self.shared_data
    
    def add_collector(self, collector):
        """Add a new collector (for extensibility)."""
        if not self.running.is_set():
            self.collectors.append(collector)
        else:
            # Start immediately if collection is already running
            thread = threading.Thread(
                target=collector.collect_loop,
                args=(self.shared_data, self.running),
                daemon=True
            )
            thread.start()
            self.threads.append(thread)
            self.collectors.append(collector)