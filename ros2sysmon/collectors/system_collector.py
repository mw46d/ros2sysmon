"""System metrics collector for CPU, memory, disk, and temperature."""
import time
import threading
import psutil
from datetime import datetime
from typing import Optional
from ..data_models import SystemMetrics, SystemAlert
from ..shared_data import SharedDataStore
from ..config import Config


class SystemCollector:
    """Collects system-level metrics like CPU, memory, disk usage."""
    
    def __init__(self, config: Config):
        """Initialize the system collector."""
        self.config = config
        self.last_network_check = time.time()
        self.manual_refresh_requested = False
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        """Main collection loop with zero-interval support."""
        # Initial collection
        self._do_collection(shared_data)
        
        # Get collection interval
        interval = self.config.collection_intervals.system_metrics
        
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
            metrics = self._collect_system_metrics(shared_data)
            shared_data.update_system_metrics(metrics)
            
            # Collect process information
            processes = self._get_ros_processes()
            shared_data.update_processes(processes)
            
            # Generate alerts based on thresholds
            alerts = self._check_thresholds(metrics)
            for alert in alerts:
                shared_data.add_alert(alert)
                
        except (psutil.NoSuchProcess, PermissionError):
            # Expected errors - just skip this iteration
            pass
    
    def trigger_manual_refresh(self):
        """Trigger an immediate collection cycle."""
        self.manual_refresh_requested = True
    
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
    
    def _get_ros_processes(self):
        """Get list of ROS-related processes."""
        try:
            ros_processes = []
            ros_keywords = [
                "ros2", "rviz", "gazebo", "navigation", "moveit", "rqt",
                "robot_state", "joint_state", "launch", "python3",
                "map_server", "nav2", "slam"
            ]
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'cpu_percent', 'memory_percent']):
                try:
                    proc_info = proc.info
                    proc_name = proc_info['name'].lower()
                    cmdline = ' '.join(proc_info['cmdline']).lower() if proc_info['cmdline'] else ''
                    
                    # Check if process is ROS-related
                    is_ros_process = any(
                        keyword in proc_name or keyword in cmdline
                        for keyword in ros_keywords
                    )
                    
                    if is_ros_process:
                        # Get CPU percentage
                        try:
                            cpu_pct = proc.cpu_percent(interval=0.01)  # Faster interval for UI
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            cpu_pct = 0.0
                            
                        ros_processes.append({
                            'pid': proc_info['pid'],
                            'name': proc_info['name'],
                            'cpu_percent': cpu_pct,
                            'memory_percent': proc_info['memory_percent'] or 0.0
                        })
                        
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                    
            return ros_processes
            
        except (OSError, PermissionError):
            return []