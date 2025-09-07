"""Display management using Textual for beautiful terminal UI."""

import time
import signal
import psutil
from datetime import datetime
from textual.app import App
from textual.containers import Vertical, Horizontal
from textual.widgets import Static, DataTable
from typing import List, Optional
from .shared_data import SharedDataStore
from .data_models import SystemAlert, SystemMetrics, ROSNodeInfo, TopicMetrics
from .config import Config


class DisplayManager(App):
    """Manages Textual-based terminal display with live updates."""
    
    CSS = """
    #header {
        height: 8;
        background: black;
        color: white;
        text-align: left;
        border-bottom: white;
    }
    
    #left {
        height: auto;
        width: 30%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #middle {
        height: auto;
        width: 70%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #left_ros {
        height: auto;
        width: 30%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #middle_ros {
        height: auto;
        width: 70%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #alerts {
        height: 5;
        background: black;
        color: white;
        text-align: left;
        border-top: white;
        dock: bottom;
    }
    
    #help {
        height: 1;
        background: black;
        color: cyan;
        text-align: left;
        dock: bottom;
    }
    
    .body {
        height: auto;
    }
    
    .hidden {
        display: none;
    }
    """

    def __init__(self, config: Config):
        """Initialize the display manager."""
        super().__init__()
        self.config = config
        self.exit_requested = False
        self.display_mode = "1"

    def compose(self):
        """Create the layout structure."""
        with Vertical():
            yield Static("Loading system metrics...", id="header")
            
            with Horizontal(classes="body"):
                # Mode 2 panes (initially hidden) - Nodes, Processes
                yield Static("ROS2 Nodes\n-----------\nROS not running", id="left", classes="hidden")
                yield Static("System Processes\n----------------\nLoading processes...", id="middle", classes="hidden")
                
                # Mode 1 panes (default visible) - Topics, TF Frames
                yield Static("ROS2 Topics\n-----------\nNo ROS topics detected", id="left_ros")
                yield Static("TF Frames\n---------\nTF monitoring coming soon", id="middle_ros")
            
            yield Static("No alerts", id="alerts")
            yield Static(self._create_help_panel(), id="help")

    def run_display(self, shared_data: SharedDataStore):
        """Run the main display with data polling loop."""
        def exit_handler(signum, frame):
            self.exit_requested = True

        signal.signal(signal.SIGTERM, exit_handler)


        # Store shared_data reference for the app
        self.shared_data = shared_data

        try:
            self.run()
        finally:
            pass

    def on_mount(self):
        """Start the data update timer when app mounts."""
        self.set_interval(self.config.refresh_rate, self._update_display)

    def on_key(self, event):
        """Handle key press events."""
        if event.key == "x":
            self.exit_requested = True
            self.exit()
        elif event.key == "1":
            self.display_mode = "1"
            self._switch_to_mode_1_panes()
        elif event.key == "2":
            self.display_mode = "2"
            self._switch_to_mode_2_panes()

    def _switch_to_mode_2_panes(self):
        """Switch to mode 2 panes (Nodes, Processes)."""
        # Show mode 2 panes
        self.query_one("#left").remove_class("hidden")
        self.query_one("#middle").remove_class("hidden")
        
        # Hide mode 1 panes
        self.query_one("#left_ros").add_class("hidden")
        self.query_one("#middle_ros").add_class("hidden")

    def _switch_to_mode_1_panes(self):
        """Switch to mode 1 panes (Topics, TF Frames)."""
        # Hide mode 2 panes
        self.query_one("#left").add_class("hidden")
        self.query_one("#middle").add_class("hidden")
        
        # Show mode 1 panes
        self.query_one("#left_ros").remove_class("hidden")
        self.query_one("#middle_ros").remove_class("hidden")

    def _update_ros_panels(self, nodes, topics):
        """Update the Mode 1 panels."""
        # Update topics panel (left in Mode 1)
        left_ros_widget = self.query_one("#left_ros")
        topics_content = f"ROS2 Topics\n-----------\n{len(topics) if topics else 0} topics detected"
        left_ros_widget.update(topics_content)
        
        # Update TF frames panel (right in Mode 1)
        middle_ros_widget = self.query_one("#middle_ros")
        tf_content = "TF Frames\n---------\nTF monitoring coming soon"
        middle_ros_widget.update(tf_content)

    def _update_display(self):
        """Update all display panels with current data."""
        if not hasattr(self, 'shared_data'):
            return
            
        try:
            metrics, nodes, topics, alerts = self.shared_data.get_system_data()
            
            # Update each panel with new data
            self._update_header_panel(metrics)
            self._update_left_panel(nodes)
            self._update_middle_panel(metrics)
            self._update_ros_panels(nodes, topics)
            self._update_alerts_panel(alerts)
            self._update_help_panel()
            
        except Exception as e:
            # Show error in header panel
            header_widget = self.query_one("#header")
            header_widget.update(f"Display Error: {str(e)}")

    def _update_header_panel(self, metrics):
        """Update the header panel with system metrics."""
        header_widget = self.query_one("#header")
        content = self._create_system_overview(metrics)
        header_widget.update(content)

    def _create_system_overview(self, metrics: Optional[SystemMetrics]) -> str:
        """Create the system overview content for header panel."""
        if not metrics:
            return "Loading system metrics..."

        # Create simple text-based progress bars
        def make_progress_bar(value: float, width: int = 15) -> str:
            filled = int(value * width / 100)
            bar = "█" * filled + "░" * (width - filled)
            return f"[{bar}] {value:5.1f}%"

        # Build content with metrics
        lines = []
        
        # System metrics row
        cpu_bar = make_progress_bar(metrics.cpu_percent)
        mem_bar = make_progress_bar(metrics.memory_percent)
        disk_bar = make_progress_bar(metrics.disk_percent)
        
        lines.append(f"CPU:    {cpu_bar}")
        lines.append(f"Memory: {mem_bar}")  
        lines.append(f"Disk:   {disk_bar}")
        
        # Additional info
        lines.append(f"Load: {metrics.load_average[0]:.2f}  Uptime: {metrics.uptime}")
        
        if metrics.temperature:
            lines.append(f"Temperature: {metrics.temperature:.1f}°C")
            
        if metrics.network_latency is not None:
            lines.append(f"Network: {metrics.network_latency:.1f}ms")
        else:
            lines.append("Network: checking...")
            
        if metrics.battery_percent:
            lines.append(f"Battery: {metrics.battery_percent:.1f}%")
            
        # Add timestamp and mode
        lines.append(f"Time: {metrics.timestamp.strftime('%H:%M:%S')} | Mode: {self.display_mode.upper()}")
        
        return "\n".join(lines)

    def _create_processes_panel(self) -> str:
        """Create the system processes panel content with ROS-related processes."""
        lines = []
        lines.append("System Processes")
        lines.append("----------------")
        lines.append("PID    Process              CPU%   Mem%")
        lines.append("-" * 40)
        
        try:
            # Get ROS-related processes
            ros_processes = []
            ros_keywords = [
                "ros2", "rviz", "gazebo", "navigation", "moveit", "rqt",
                "robot_state", "joint_state", "launch", "python3"
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
                            cpu_pct = proc.cpu_percent(interval=0.1)
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
            
            # Sort by CPU usage (highest first) and take top processes
            top_processes = sorted(
                ros_processes, 
                key=lambda p: p['cpu_percent'], 
                reverse=True
            )[:15]  # Show top 15 processes
            
            if not top_processes:
                lines.append("No ROS processes found")
            else:
                for proc in top_processes:
                    # Format process name to fit in available space
                    proc_name = proc['name'][:16] if len(proc['name']) > 16 else proc['name']
                    proc_name = proc_name.ljust(16)
                    
                    # Format data columns
                    pid_str = str(proc['pid']).ljust(6)
                    cpu_str = f"{proc['cpu_percent']:5.1f}"
                    mem_str = f"{proc['memory_percent']:5.1f}"
                    
                    line = f"{pid_str} {proc_name} {cpu_str} {mem_str}"
                    lines.append(line)
                    
        except Exception:
            lines.append("Error loading processes")
            
        return "\n".join(lines)

    def _create_network_panel(self, metrics) -> str:
        """Create the network stats panel content."""
        lines = []
        lines.append("Network Stats")
        lines.append("-------------")
        if metrics and metrics.network_latency is not None:
            lines.append(f"Ping: {metrics.network_latency:.1f}ms")
        else:
            lines.append("Ping: checking...")
        lines.append("Interfaces: Loading...")
        return "\n".join(lines)

    def _create_extra_panel(self, metrics) -> str:
        """Create the system extra panel content."""
        lines = []
        lines.append("System Extra")
        lines.append("------------")
        if metrics and metrics.temperature:
            lines.append(f"Temperature: {metrics.temperature:.1f}°C")
        if metrics and metrics.battery_percent:
            lines.append(f"Battery: {metrics.battery_percent:.1f}%")
        lines.append("More info: Loading...")
        return "\n".join(lines)

    def _update_left_panel(self, nodes):
        """Update the left panel (Mode 2: ROS Nodes)."""
        left_widget = self.query_one("#left")
        content = self._create_nodes_panel(nodes)
        left_widget.update(content)

    def _update_middle_panel(self, metrics):
        """Update the middle panel (Mode 2: System Processes)."""
        middle_widget = self.query_one("#middle")
        content = self._create_processes_panel()
        middle_widget.update(content)


    def _update_alerts_panel(self, alerts):
        """Update the alerts panel."""
        alerts_widget = self.query_one("#alerts")
        content = self._create_alerts_panel(alerts)
        alerts_widget.update(content)

    def _create_alerts_panel(self, alerts: List[SystemAlert]) -> str:
        """Create the alerts panel content."""
        if not alerts:
            return "No alerts"

        # Show last few alerts (fit within panel height)
        recent_alerts = list(alerts)[-6:]
        alert_lines = []

        for alert in recent_alerts:
            # Format time and message
            time_str = alert.timestamp.strftime("%H:%M:%S")
            level_indicator = {
                "ERROR": "ERR", 
                "WARN": "WRN", 
                "INFO": "INF"
            }.get(alert.level, alert.level)
            
            # Create alert line (keep it short for display)
            alert_line = f"[{level_indicator}] {time_str} - {alert.message}"
            
            # Truncate if too long
            if len(alert_line) > 50:
                alert_line = alert_line[:47] + "..."
                
            alert_lines.append(alert_line)
        
        return "\n".join(alert_lines)

    def _update_help_panel(self):
        """Update the help panel."""
        help_widget = self.query_one("#help")
        content = self._create_help_panel()
        help_widget.update(content)

    def _create_help_panel(self) -> str:
        """Create help panel content with keyboard shortcuts."""
        return "Commands: '1' = Mode 1, '2' = Mode 2, 'x' = quit"

    def _create_nodes_panel(self, nodes: List[ROSNodeInfo]) -> str:
        """Create the ROS nodes panel content with just node names."""
        if not nodes:
            return "ROS2 Nodes\n-----------\nROS not running"

        lines = []
        lines.append(f"Nodes ({len(nodes)})")
        lines.append("-" * 20)

        # Sort nodes alphabetically for consistent display
        sorted_nodes = sorted(nodes, key=lambda n: n.name.lower())

        # Show all nodes, truncate to fit wider panel (30% width)
        for node in sorted_nodes:
            # Clean up node name display and truncate to 30 chars for wider column
            node_name = node.name.strip()
            if len(node_name) > 30:
                node_name = node_name[:27] + "..."
            lines.append(node_name)

        return "\n".join(lines)