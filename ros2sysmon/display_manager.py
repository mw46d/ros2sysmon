"""Display management using Textual for beautiful terminal UI."""

import time
import signal
import psutil
from datetime import datetime
from textual.app import App
from textual.containers import Vertical, Horizontal
from textual.widgets import Static
from typing import List, Optional
from .shared_data import SharedDataStore
from .data_models import SystemAlert, SystemMetrics, ROSNodeInfo, TopicMetrics
from .config import Config
from .keyboard_handler import KeyboardHandler


class DisplayManager(App):
    """Manages Textual-based terminal display with live updates."""
    
    CSS = """
    #header {
        height: 7;
        background: black;
        color: red;
        text-align: left;
    }
    
    #left {
        height: auto;
        width: 1fr;
        background: black;
        color: green;
        text-align: left;
    }
    
    #middle {
        height: auto;
        width: 1fr;
        background: black;
        color: blue;
        text-align: left;
    }
    
    #right {
        height: auto;
        width: 1fr;
        background: black;
        color: purple;
        text-align: left;
    }
    
    #alerts {
        height: 8;
        background: black;
        color: orange;
        text-align: left;
        border: white;
    }
    
    #help {
        height: 3;
        background: black;
        color: cyan;
        text-align: left;
    }
    
    .body {
        height: auto;
        border: solid white;
    }
    """

    def __init__(self, config: Config):
        """Initialize the display manager."""
        super().__init__()
        self.config = config
        self.exit_requested = False
        self.display_mode = "system"
        self.keyboard_handler = KeyboardHandler()

    def compose(self):
        """Create the layout structure."""
        with Vertical():
            yield Static("Header disabled for testing", id="header")
            
            with Horizontal(classes="body"):
                yield Static("Left pane disabled", id="left")
                yield Static("Middle pane disabled", id="middle")
                yield Static("Right pane disabled", id="right")
            
            yield Static("Alerts pane disabled", id="alerts")
            yield Static("Help pane disabled", id="help")

    def run_display(self, shared_data: SharedDataStore):
        """Run the main display with data polling loop."""
        def exit_handler(signum, frame):
            self.exit_requested = True

        signal.signal(signal.SIGTERM, exit_handler)

        self.keyboard_handler.set_exit_callback(
            lambda: setattr(self, "exit_requested", True)
        )
        self.keyboard_handler.start_listening()

        # Store shared_data reference for the app
        self.shared_data = shared_data

        try:
            self.run()
        finally:
            self.keyboard_handler.stop_listening()

    def on_mount(self):
        """Start the data update timer when app mounts."""
        self.set_interval(self.config.refresh_rate, self._update_display)

    def on_key(self, event):
        """Handle key press events."""
        if event.key == "x":
            self.exit_requested = True
            self.exit()

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
            self._update_right_panel(topics, metrics)
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

    def _update_left_panel(self, nodes):
        """Update the left panel."""
        left_widget = self.query_one("#left")
        content = self._create_nodes_panel(nodes)
        left_widget.update(content)

    def _update_middle_panel(self, metrics):
        """Update the middle panel."""
        middle_widget = self.query_one("#middle")
        content = "System Details"
        middle_widget.update(content)

    def _update_right_panel(self, topics, metrics):
        """Update the right panel."""
        right_widget = self.query_one("#right")
        content = f"ROS Topics: {len(topics) if topics else 0}"
        right_widget.update(content)

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
        
        alert_lines.append(f"System Alerts ({len(alerts)} total)")
        alert_lines.append("-" * 30)

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
        help_lines = []
        
        help_lines.append("Help")
        help_lines.append("Commands: Press 'x' or Ctrl+C to quit")
        
        return "\n".join(help_lines)

    def _create_nodes_panel(self, nodes: List[ROSNodeInfo]) -> str:
        """Create the ROS nodes panel content for left panel."""
        if not nodes:
            return "ROS2 Nodes\n-----------\nNo ROS nodes detected"

        lines = []
        lines.append(f"ROS2 Nodes ({len(nodes)} total)")
        lines.append("-" * 25)
        
        # Table headers
        lines.append("Node                     CPU%  Mem(MB) Status Uptime")
        lines.append("-" * 50)

        # Sort nodes by CPU usage (highest first)
        sorted_nodes = sorted(nodes, key=lambda n: n.cpu_percent, reverse=True)

        # Show top 10 nodes to fit in panel
        for node in sorted_nodes[:10]:
            # Truncate long node names
            node_name = node.name[:20] if len(node.name) > 20 else node.name
            node_name = node_name.ljust(20)
            
            # Format data columns
            cpu_str = f"{node.cpu_percent:5.1f}"
            mem_str = f"{node.memory_mb:7.1f}"
            status_str = node.status[:6].ljust(6)
            uptime_str = node.uptime[:8] if len(node.uptime) > 8 else node.uptime.ljust(8)
            
            line = f"{node_name} {cpu_str} {mem_str} {status_str} {uptime_str}"
            lines.append(line)

        return "\n".join(lines)