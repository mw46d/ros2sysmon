"""Display management using Rich for beautiful terminal UI."""
import time
from rich.console import Console
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.progress import Progress, BarColumn, TextColumn, TimeElapsedColumn
from rich.table import Table
from rich.text import Text
from datetime import datetime
from typing import List, Optional
from .shared_data import SharedDataStore
from .models import SystemMetrics, SystemAlert
from .config import Config


class DisplayManager:
    """Manages Rich-based terminal display with live updates."""
    
    def __init__(self, config: Config):
        """Initialize the display manager."""
        self.config = config
        self.console = Console()
        self.layout = self._create_layout()
    
    def run_display(self, shared_data: SharedDataStore):
        """Run the main display loop with Rich Live."""
        with Live(
            self.layout, 
            console=self.console, 
            refresh_per_second=1/self.config.refresh_rate,
            screen=True  # Full screen mode
        ) as live:
            try:
                while True:
                    self._update_display(shared_data)
                    time.sleep(self.config.refresh_rate)
            except KeyboardInterrupt:
                pass
    
    def _create_layout(self) -> Layout:
        """Create the main layout structure."""
        layout = Layout()
        
        layout.split_column(
            Layout(name="header", size=8),  # Increased from 5 to 8 for more system info
            Layout(name="body"),
            Layout(name="alerts", size=4)
        )
        
        layout["body"].split_row(
            Layout(name="left"),
            Layout(name="right")
        )
        
        return layout
    
    def _update_display(self, shared_data: SharedDataStore):
        """Update all display panels with current data."""
        metrics, nodes, topics, alerts = shared_data.get_system_data()
        
        # Update header with system overview
        self.layout["header"].update(
            self._create_system_overview(metrics)
        )
        
        # Update left panel with placeholder content for now
        self.layout["left"].update(
            Panel("ROS Nodes\n(Coming soon)", title="ROS2 Nodes", border_style="blue")
        )
        
        # Update right panel with placeholder content for now  
        self.layout["right"].update(
            Panel("Topics\n(Coming soon)", title="Topics", border_style="green")
        )
        
        # Update alerts panel
        self.layout["alerts"].update(
            self._create_alerts_panel(alerts)
        )
    
    def _create_system_overview(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create the system overview panel with progress bars in two columns."""
        if not metrics:
            return Panel("Loading system metrics...", title="System Resources", border_style="blue")
        
        # Create simple text-based progress bars
        def make_progress_bar(value: float, width: int = 15) -> str:
            filled = int(value * width / 100)
            bar = "█" * filled + "░" * (width - filled)
            return f"[{bar}] {value:5.1f}%"
        
        # Build left and right columns manually with string formatting
        left_col_width = 35  # Space for progress bars
        right_col = []
        
        # Left column data
        left_data = [
            f"CPU:    {make_progress_bar(metrics.cpu_percent)}",
            f"Memory: {make_progress_bar(metrics.memory_percent)}",
            f"Disk:   {make_progress_bar(metrics.disk_percent)}"
        ]
        
        # Right column data
        right_data = [
            f"Load: {metrics.load_average[0]:.2f}  Uptime: {metrics.uptime}"
        ]
        
        if metrics.temperature:
            right_data.append(f"Temperature: {metrics.temperature:.1f}°C")
            
        if metrics.network_latency is not None:
            right_data.append(f"Network: {metrics.network_latency:.1f}ms")
        else:
            right_data.append("Network: checking...")
            
        if metrics.battery_percent:
            right_data.append(f"Battery: {metrics.battery_percent:.1f}%")
        
        # Combine into two-column format
        content_lines = []
        max_lines = max(len(left_data), len(right_data))
        
        for i in range(max_lines):
            left_part = left_data[i] if i < len(left_data) else ""
            right_part = right_data[i] if i < len(right_data) else ""
            
            # Format as two columns
            line = f"{left_part:<{left_col_width}} | {right_part}"
            content_lines.append(line)
        
        return Panel(
            "\n".join(content_lines),
            title=f"System Resources - {metrics.timestamp.strftime('%H:%M:%S')}",
            border_style="blue"
        )
    
    def _create_alerts_panel(self, alerts: List[SystemAlert]) -> Panel:
        """Create the alerts panel."""
        if not alerts:
            return Panel("No alerts", title="System Alerts", border_style="green")
        
        # Show last few alerts
        recent_alerts = list(alerts)[-3:]
        alert_text = []
        
        for alert in recent_alerts:
            style = {
                "ERROR": "red",
                "WARN": "yellow", 
                "INFO": "blue"
            }.get(alert.level, "white")
            
            time_str = alert.timestamp.strftime('%H:%M:%S')
            alert_text.append(
                f"[{style}][{alert.level}][/{style}] {time_str} - {alert.message}"
            )
        
        border_style = "red" if any(a.level == "ERROR" for a in recent_alerts) else "yellow"
        
        return Panel(
            "\n".join(alert_text),
            title=f"System Alerts ({len(alerts)} total)",
            border_style=border_style
        )