"""System metrics collector for CPU, memory, disk, and temperature."""
import time
import threading
import psutil
from datetime import datetime
from typing import Optional
from ..models import SystemMetrics, SystemAlert
from ..shared_data import SharedDataStore
from ..config import Config


class SystemCollector:
    """Collects system-level metrics like CPU, memory, disk usage."""
    
    def __init__(self, config: Config):
        """Initialize the system collector."""
        self.config = config
        self.last_network_check = time.time()
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        """Main collection loop - runs in background thread."""
        while running.is_set():
            try:
                metrics = self._collect_system_metrics(shared_data)
                shared_data.update_system_metrics(metrics)
                
                # Generate alerts based on thresholds
                alerts = self._check_thresholds(metrics)
                for alert in alerts:
                    shared_data.add_alert(alert)
                    
            except (psutil.NoSuchProcess, PermissionError):
                # Expected errors - just skip this iteration
                pass
            
            time.sleep(self.config.refresh_rate)
    
    def _collect_system_metrics(self, shared_data: SharedDataStore) -> SystemMetrics:
        """Collect current system metrics, preserving network latency."""
        # Get existing network latency to preserve it
        existing_latency = None
        with shared_data._lock:
            if shared_data.system_metrics:
                existing_latency = shared_data.system_metrics.network_latency
        
        return SystemMetrics(
            cpu_percent=psutil.cpu_percent(interval=0.1),
            memory_percent=psutil.virtual_memory().percent,
            disk_percent=psutil.disk_usage('/').percent,
            temperature=self._get_temperature(),
            load_average=psutil.getloadavg(),
            uptime=self._get_uptime(),
            battery_percent=self._get_battery(),
            network_latency=existing_latency,  # Preserve existing value
            timestamp=datetime.now()
        )
    
    def _get_temperature(self) -> Optional[float]:
        """Get system temperature from available sensors."""
        try:
            temps = psutil.sensors_temperatures()
            if 'coretemp' in temps:
                return temps['coretemp'][0].current
            if 'cpu_thermal' in temps:
                return temps['cpu_thermal'][0].current
        except:
            pass
        return None
    
    def _get_uptime(self) -> str:
        """Get system uptime as formatted string."""
        boot_time = psutil.boot_time()
        uptime_seconds = time.time() - boot_time
        hours = int(uptime_seconds // 3600)
        minutes = int((uptime_seconds % 3600) // 60)
        return f"{hours:02d}:{minutes:02d}"
    
    def _get_battery(self) -> Optional[float]:
        """Get battery percentage if available."""
        try:
            battery = psutil.sensors_battery()
            return battery.percent if battery else None
        except:
            return None
    
    def _check_thresholds(self, metrics: SystemMetrics) -> list[SystemAlert]:
        """Check metrics against thresholds and generate alerts."""
        alerts = []
        
        # CPU alerts
        if metrics.cpu_percent > self.config.thresholds.cpu_error:
            alerts.append(SystemAlert("ERROR", f"CPU usage critical: {metrics.cpu_percent:.1f}%", 
                                    datetime.now(), "SYSTEM"))
        elif metrics.cpu_percent > self.config.thresholds.cpu_warn:
            alerts.append(SystemAlert("WARN", f"CPU usage high: {metrics.cpu_percent:.1f}%",
                                    datetime.now(), "SYSTEM"))
        
        # Memory alerts  
        if metrics.memory_percent > self.config.thresholds.memory_error:
            alerts.append(SystemAlert("ERROR", f"Memory usage critical: {metrics.memory_percent:.1f}%",
                                    datetime.now(), "SYSTEM"))
        elif metrics.memory_percent > self.config.thresholds.memory_warn:
            alerts.append(SystemAlert("WARN", f"Memory usage high: {metrics.memory_percent:.1f}%",
                                    datetime.now(), "SYSTEM"))
        
        return alerts