"""Network latency collector for monitoring connectivity."""
import time
import threading
import subprocess
import re
from datetime import datetime
from typing import Optional
from ..data_models import SystemAlert
from ..shared_data import SharedDataStore
from ..config import Config


class NetworkCollector:
    """Collects network latency metrics via ping."""
    
    def __init__(self, config: Config):
        """Initialize the network collector."""
        self.config = config
        self.last_ping_time = 0
        self.manual_refresh_requested = False
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        """Main collection loop with zero-interval support."""
        # Initial collection
        self._do_collection(shared_data)
        
        # Get collection interval
        interval = self.config.collection_intervals.network_ping
        
        if interval == 0.0:
            # Zero interval: run once and wait for manual refresh
            while running.is_set():
                if self.manual_refresh_requested:
                    self.manual_refresh_requested = False
                    self._do_collection(shared_data)
                time.sleep(0.1)  # Small sleep to avoid busy waiting
        else:
            # Normal interval-based collection
            while running.is_set():
                time.sleep(interval)
                if running.is_set():  # Check again after sleep
                    self._do_collection(shared_data)
    
    def _do_collection(self, shared_data: SharedDataStore):
        """Perform one collection cycle."""
        try:
            latency = self._check_network_latency()
            
            # Only update if we got a valid measurement
            if latency is not None:
                self._update_system_metrics_with_latency(shared_data, latency)
                
                # Generate network alerts
                alerts = self._check_latency_thresholds(latency)
                for alert in alerts:
                    shared_data.add_alert(alert)
            
        except Exception:
            # Network issues expected - just skip this check
            pass
    
    def trigger_manual_refresh(self):
        """Trigger an immediate collection cycle."""
        self.manual_refresh_requested = True
    
    def _check_network_latency(self) -> Optional[float]:
        """Check network latency with ping to 8.8.8.8."""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '2', '8.8.8.8'], 
                capture_output=True, 
                timeout=3,
                text=True
            )
            
            if result.returncode == 0:
                return self._parse_ping_time(result.stdout)
                
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            # Network down, timeout, or ping not available
            pass
        
        return None
    
    def _parse_ping_time(self, ping_output: str) -> Optional[float]:
        """Parse ping time from ping command output."""
        # Look for "time=X.XXXms" pattern
        match = re.search(r'time=([0-9.]+)\s*ms', ping_output)
        if match:
            return float(match.group(1))
        return None
    
    def _update_system_metrics_with_latency(self, shared_data: SharedDataStore, latency: Optional[float]):
        """Update existing system metrics with network latency."""
        with shared_data._lock:
            if shared_data.system_metrics:
                # Create a copy and update latency
                metrics = shared_data.system_metrics
                from dataclasses import replace
                shared_data.system_metrics = replace(metrics, network_latency=latency)
    
    def _check_latency_thresholds(self, latency: float) -> list[SystemAlert]:
        """Check network latency against thresholds."""
        alerts = []
        
        if latency > self.config.thresholds.network_latency_error:
            alerts.append(SystemAlert(
                "ERROR", 
                f"Network latency critical: {latency:.1f}ms", 
                datetime.now(), 
                "NETWORK"
            ))
        elif latency > self.config.thresholds.network_latency_warn:
            alerts.append(SystemAlert(
                "WARN", 
                f"Network latency high: {latency:.1f}ms",
                datetime.now(), 
                "NETWORK"
            ))
        
        return alerts