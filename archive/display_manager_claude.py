"""Display management using Rich for beautiful terminal UI."""

import time
import signal
import psutil
from rich.console import Console
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.progress import Progress, BarColumn, TextColumn, TimeElapsedColumn
from rich.table import Table
from rich.text import Text
from rich import box
from datetime import datetime
from typing import List, Optional, Dict, Any
from .shared_data import SharedDataStore
from .data_models import SystemAlert, SystemMetrics, ROSNodeInfo, TopicMetrics
from .config import Config
from .keyboard_handler import KeyboardHandler


class ColorScheme:
    """Centralized color definitions for consistent theming."""
    
    # Status colors
    STATUS_OK = "green"
    STATUS_WARN = "yellow"
    STATUS_ERROR = "red"
    STATUS_UNKNOWN = "white"
    
    # Performance colors
    PERF_HIGH = "red"
    PERF_MED = "yellow"
    PERF_LOW = "green"
    
    # UI colors
    HEADER = "bold blue"
    ACCENT = "cyan"
    SECONDARY = "magenta"
    INFO = "blue"
    
    @classmethod
    def get_status_color(cls, status: str) -> str:
        """Get color for status strings."""
        return {
            "OK": cls.STATUS_OK,
            "WARN": cls.STATUS_WARN,
            "ERROR": cls.STATUS_ERROR,
            "SLOW": cls.STATUS_WARN,
            "IDLE": cls.STATUS_ERROR,
            "running": cls.STATUS_OK
        }.get(status, cls.STATUS_UNKNOWN)
    
    @classmethod
    def get_performance_color(cls, value: float, high_threshold: float = 80, med_threshold: float = 50) -> str:
        """Get color based on performance value."""
        if value > high_threshold:
            return cls.PERF_HIGH
        elif value > med_threshold:
            return cls.PERF_MED
        else:
            return cls.PERF_LOW


class DisplayManager:
    """Manages Rich-based terminal display with live updates."""

    def __init__(self, config: Config):
        """Initialize the display manager."""
        self.config = config
        self.console = Console()
        self.layout = self._create_layout()
        self.exit_requested = False
        self.display_mode = "system"  # Default mode
        self.keyboard_handler = KeyboardHandler()
        
        # Performance optimization - cache panels
        self._cached_panels: Dict[str, Any] = {}
        self._last_data_hash: Dict[str, str] = {}

    def run_display(self, shared_data: SharedDataStore):
        """Run the main display loop."""
        def exit_handler(signum, frame):
            self.exit_requested = True

        signal.signal(signal.SIGTERM, exit_handler)

        self.keyboard_handler.set_exit_callback(
            lambda: setattr(self, "exit_requested", True)
        )
        self.keyboard_handler.start_listening()

        try:
            with Live(
                self.layout,
                console=self.console,
                refresh_per_second=1 / self.config.refresh_rate,
                screen=True,
            ) as live:
                try:
                    while not self.exit_requested:
                        self._update_display(shared_data)
                        time.sleep(self.config.refresh_rate)
                except KeyboardInterrupt:
                    pass
        finally:
            self.keyboard_handler.stop_listening()

    def _create_layout(self) -> Layout:
        """Create the main layout structure."""
        layout = Layout()

        layout.split_column(
            Layout(name="header", size=7),
            Layout(name="body"),
            Layout(name="alerts", size=8),
            Layout(name="help", size=3),
        )

        layout["body"].split_row(
            Layout(name="left"), Layout(name="middle"), Layout(name="right")
        )

        return layout

    def _create_base_table(self, header_style: str = None) -> Table:
        """Create a base table with consistent styling."""
        return Table(
            show_header=True,
            header_style=header_style or ColorScheme.HEADER,
            expand=True,
            show_lines=False,
            padding=0,
            box=box.SIMPLE
        )

    def _make_progress_bar(self, value: float, width: int = 15) -> str:
        """Create a Rich-style progress bar with color coding."""
        filled = int(value * width / 100)
        bar = "█" * filled + "░" * (width - filled)
        color = ColorScheme.get_performance_color(value)
        return f"[{color}]{bar}[/{color}]"

    def _should_update_panel(self, panel_name: str, data: Any) -> bool:
        """Check if panel needs updating based on data changes."""
        # Simple hash-based comparison for performance
        data_hash = str(hash(str(data)))
        if self._last_data_hash.get(panel_name) != data_hash:
            self._last_data_hash[panel_name] = data_hash
            return True
        return False

    def _update_display(self, shared_data: SharedDataStore):
        """Update all display panels with current data."""
        try:
            metrics, nodes, topics, alerts = shared_data.get_system_data()

            self._update_header_panel(metrics)
            self._update_left_panel(nodes)
            self._update_middle_panel(metrics)
            self._update_right_panel(topics, metrics)
            self._update_alerts_panel(alerts)
            self._update_help_panel()
            
        except Exception as e:
            error_panel = Panel(
                f"[{ColorScheme.STATUS_ERROR}]Display Error: {str(e)}[/{ColorScheme.STATUS_ERROR}]",
                title=f"[{ColorScheme.STATUS_ERROR}]Error[/{ColorScheme.STATUS_ERROR}]",
                border_style=ColorScheme.STATUS_ERROR
            )
            self.layout["header"].update(error_panel)

    def _update_header_panel(self, metrics):
        """Update the header panel."""
        # For testing - disable header
        self.layout["header"].update(
            Panel("Header disabled for testing", title="Header")
        )

    def _update_left_panel(self, nodes):
        """Update the left panel."""
        self.layout["left"].update(
            Panel("Left pane disabled", title="Left")
        )

    def _update_middle_panel(self, metrics):
        """Update the middle panel."""
        self.layout["middle"].update(
            Panel("Middle pane disabled", title="Middle")
        )

    def _update_right_panel(self, topics, metrics):
        """Update the right panel."""
        self.layout["right"].update(
            Panel("Right pane disabled", title="Right")
        )

    def _update_alerts_panel(self, alerts):
        """Update the alerts panel."""
        self.layout["alerts"].update(self._create_alerts_panel(alerts))

    def _update_help_panel(self):
        """Update the help panel."""
        self.layout["help"].update(self._create_help_panel())

    def _create_system_overview(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create the system overview panel with Rich components."""
        if not metrics:
            return Panel("Loading system metrics...", title="System Resources")

        # Create main metrics table
        metrics_table = Table(show_header=False, box=None, padding=0, expand=True)
        metrics_table.add_column("Metric", style="bold", min_width=8)
        metrics_table.add_column("Progress", ratio=2)
        metrics_table.add_column("Value", justify="right", min_width=8)

        # Add system metrics with progress bars
        metrics_table.add_row(
            "CPU:",
            self._make_progress_bar(metrics.cpu_percent),
            f"{metrics.cpu_percent:.1f}%"
        )
        metrics_table.add_row(
            "Memory:",
            self._make_progress_bar(metrics.memory_percent),
            f"{metrics.memory_percent:.1f}%"
        )
        metrics_table.add_row(
            "Disk:",
            self._make_progress_bar(metrics.disk_percent),
            f"{metrics.disk_percent:.1f}%"
        )

        # Create status info table
        status_table = Table(show_header=False, box=None, padding=0, expand=True)
        status_table.add_column("Info", style=ColorScheme.ACCENT)

        status_table.add_row(f"Load: {metrics.load_average[0]:.2f}  Uptime: {metrics.uptime}")

        if metrics.temperature:
            temp_color = ColorScheme.get_performance_color(metrics.temperature, 70, 50)
            status_table.add_row(f"Temperature: [{temp_color}]{metrics.temperature:.1f}°C[/{temp_color}]")

        if metrics.network_latency is not None:
            latency_color = ColorScheme.get_performance_color(metrics.network_latency, 100, 50)
            status_table.add_row(f"Network: [{latency_color}]{metrics.network_latency:.1f}ms[/{latency_color}]")
        else:
            status_table.add_row("Network: checking...")

        if metrics.battery_percent:
            batt_color = ColorScheme.get_performance_color(100 - metrics.battery_percent, 80, 50)  # Invert for battery
            status_table.add_row(f"Battery: [{batt_color}]{metrics.battery_percent:.1f}%[/{batt_color}]")

        # Combine tables in layout
        main_layout = Table(show_header=False, box=None, padding=0, expand=True)
        main_layout.add_column("Metrics", ratio=1)
        main_layout.add_column("Status", ratio=1)
        main_layout.add_row(metrics_table, status_table)

        # Add mode indicator
        mode_text = f"\nMode: [{ColorScheme.ACCENT}]{self.display_mode.upper()}[/{ColorScheme.ACCENT}]"

        # Combine everything
        content = Table(show_header=False, box=None, padding=0)
        content.add_column("Content")
        content.add_row(main_layout)
        content.add_row(mode_text)

        return Panel(
            content,
            title=f"System Resources - {metrics.timestamp.strftime('%H:%M:%S')}",
            border_style=ColorScheme.ACCENT
        )

    def _create_alerts_panel(self, alerts: List[SystemAlert]) -> Panel:
        """Create the alerts panel with Rich formatting."""
        if not alerts:
            return Panel(
                f"[{ColorScheme.STATUS_OK}]No alerts[/{ColorScheme.STATUS_OK}]",
                title="System Alerts",
                border_style=ColorScheme.STATUS_OK
            )

        # Show recent alerts
        recent_alerts = list(alerts)[-6:]
        
        alert_table = Table(show_header=False, box=None, padding=0, expand=True)
        alert_table.add_column("Level", min_width=6)
        alert_table.add_column("Time", min_width=8)
        alert_table.add_column("Message", ratio=1)

        for alert in recent_alerts:
            level_color = ColorScheme.get_status_color(alert.level)
            time_str = alert.timestamp.strftime("%H:%M:%S")
            
            alert_table.add_row(
                f"[{level_color}]{alert.level}[/{level_color}]",
                time_str,
                alert.message
            )

        # Determine border color based on alert levels
        border_style = ColorScheme.STATUS_OK
        if any(a.level == "ERROR" for a in recent_alerts):
            border_style = ColorScheme.STATUS_ERROR
        elif any(a.level == "WARN" for a in recent_alerts):
            border_style = ColorScheme.STATUS_WARN

        return Panel(
            alert_table,
            title=f"System Alerts ({len(alerts)} total)",
            border_style=border_style
        )

    def _create_nodes_panel(self, nodes: List[ROSNodeInfo]) -> Panel:
        """Create the ROS nodes panel with Rich table."""
        if not nodes:
            return Panel(
                f"[{ColorScheme.STATUS_WARN}]No ROS nodes detected[/{ColorScheme.STATUS_WARN}]",
                title="ROS2 Nodes",
                border_style=ColorScheme.STATUS_WARN
            )

        table = self._create_base_table()
        table.add_column("Node", style=ColorScheme.ACCENT, ratio=3)
        table.add_column("CPU%", justify="right", min_width=6)
        table.add_column("Mem(MB)", justify="right", min_width=8)
        table.add_column("Status", min_width=6)
        table.add_column("Uptime", min_width=8)

        # Sort nodes by CPU usage
        sorted_nodes = sorted(nodes, key=lambda n: n.cpu_percent, reverse=True)

        for node in sorted_nodes[:10]:
            cpu_color = ColorScheme.get_performance_color(node.cpu_percent, 50, 10)
            status_color = ColorScheme.get_status_color(node.status)

            table.add_row(
                node.name,
                f"[{cpu_color}]{node.cpu_percent:.1f}[/{cpu_color}]",
                f"{node.memory_mb:.1f}",
                f"[{status_color}]{node.status}[/{status_color}]",
                node.uptime,
            )

        return Panel(table, title=f"ROS2 Nodes ({len(nodes)} total)")

    def _create_topics_panel(self, topics: List[TopicMetrics]) -> Panel:
        """Create the ROS topics panel with Rich table."""
        if not topics:
            return Panel(
                f"[{ColorScheme.STATUS_WARN}]No ROS topics detected[/{ColorScheme.STATUS_WARN}]",
                title="ROS2 Topics",
                border_style=ColorScheme.STATUS_WARN
            )

        table = self._create_base_table()
        table.add_column("Topic", style=ColorScheme.ACCENT, ratio=2)
        table.add_column("Hz", justify="right", min_width=6)
        table.add_column("Target", justify="right", min_width=6)
        table.add_column("Status", min_width=6)
        table.add_column("Type", ratio=1)

        # Sort topics by frequency
        sorted_topics = sorted(topics, key=lambda t: t.frequency_hz, reverse=True)

        for topic in sorted_topics[:10]:
            status_color = ColorScheme.get_status_color(topic.status)

            table.add_row(
                topic.name,
                f"{topic.frequency_hz:.1f}",
                f"{topic.target_frequency:.0f}",
                f"[{status_color}]{topic.status}[/{status_color}]",
                topic.msg_type,
            )

        return Panel(table, title=f"ROS2 Topics ({len(topics)} total)")

    def _create_processes_panel(self) -> Panel:
        """Create processes panel for system mode."""
        table = self._create_base_table()
        table.add_column("PID", justify="right", min_width=6)
        table.add_column("Process", ratio=2)
        table.add_column("CPU%", justify="right", min_width=6)
        table.add_column("Mem%", justify="right", min_width=6)
        table.add_column("Status", min_width=6)

        try:
            # Get ROS-related processes
            ros_processes = []
            ros_keywords = [
                "ros2", "rviz", "gazebo", "navigation", "moveit", "rqt",
                "robot_state_publisher", "joint_state_publisher",
            ]

            for proc in psutil.process_iter(
                ["pid", "name", "cmdline", "cpu_percent", "memory_percent", "status"]
            ):
                try:
                    proc_info = proc.info
                    proc_name = proc_info["name"].lower()
                    cmdline = (
                        " ".join(proc_info["cmdline"]).lower()
                        if proc_info["cmdline"]
                        else ""
                    )

                    # Check if process is ROS-related
                    is_ros_process = any(
                        keyword in proc_name or keyword in cmdline
                        for keyword in ros_keywords
                    )

                    if is_ros_process:
                        ros_processes.append(proc_info)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            # Sort by CPU usage
            top_processes = sorted(
                ros_processes, key=lambda p: p["cpu_percent"] or 0, reverse=True
            )[:12]

            for proc in top_processes:
                cpu_pct = proc["cpu_percent"] or 0.0
                mem_pct = proc["memory_percent"] or 0.0

                cpu_color = ColorScheme.get_performance_color(cpu_pct, 50, 10)
                status_color = ColorScheme.get_status_color(proc["status"])

                table.add_row(
                    str(proc["pid"]),
                    proc["name"][:30],
                    f"[{cpu_color}]{cpu_pct:.1f}[/{cpu_color}]",
                    f"{mem_pct:.1f}",
                    f"[{status_color}]{proc['status']}[/{status_color}]",
                )

        except Exception as e:
            return Panel(
                f"[{ColorScheme.STATUS_ERROR}]Error loading ROS processes: {str(e)}[/{ColorScheme.STATUS_ERROR}]",
                title="ROS Processes",
                border_style=ColorScheme.STATUS_ERROR
            )

        return Panel(table, title="ROS Processes (Top CPU)")

    def _create_system_details_panel(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create system details panel for system mode."""
        if not metrics:
            return Panel(
                "Loading system details...",
                title="System Details",
                border_style=ColorScheme.INFO
            )

        details_table = Table(show_header=False, box=None, padding=0, expand=True)
        details_table.add_column("Section", style="bold")

        try:
            # CPU Information
            cpu_count = psutil.cpu_count(logical=True)
            cpu_freq = psutil.cpu_freq()
            load_1, load_5, load_15 = metrics.load_average

            cpu_info = f"[{ColorScheme.ACCENT}]CPU Information[/{ColorScheme.ACCENT}]\n"
            cpu_info += f"Cores: {cpu_count} logical\n"
            if cpu_freq:
                cpu_info += f"Frequency: {cpu_freq.current:.0f} MHz\n"
            cpu_info += f"Load Avg: {load_1:.2f}, {load_5:.2f}, {load_15:.2f}"

            details_table.add_row(cpu_info)

            # Memory Information
            vm = psutil.virtual_memory()
            swap = psutil.swap_memory()

            mem_info = f"\n[{ColorScheme.STATUS_OK}]Memory Information[/{ColorScheme.STATUS_OK}]\n"
            mem_info += f"Total: {vm.total / (1024**3):.1f} GB\n"
            mem_info += f"Available: {vm.available / (1024**3):.1f} GB\n"
            mem_info += f"Used: {vm.used / (1024**3):.1f} GB ({vm.percent:.1f}%)\n"
            mem_info += f"Swap: {swap.used / (1024**3):.1f} GB / {swap.total / (1024**3):.1f} GB"

            details_table.add_row(mem_info)

            # Disk Information
            disk = psutil.disk_usage("/")
            disk_info = f"\n[{ColorScheme.SECONDARY}]Disk Information[/{ColorScheme.SECONDARY}]\n"
            disk_info += f"Total: {disk.total / (1024**3):.1f} GB\n"
            disk_info += f"Used: {disk.used / (1024**3):.1f} GB ({metrics.disk_percent:.1f}%)\n"
            disk_info += f"Free: {disk.free / (1024**3):.1f} GB"

            details_table.add_row(disk_info)

        except Exception as e:
            return Panel(
                f"[{ColorScheme.STATUS_ERROR}]Error loading system details: {str(e)}[/{ColorScheme.STATUS_ERROR}]",
                title="System Details",
                border_style=ColorScheme.STATUS_ERROR
            )

        return Panel(details_table, title="System Details")

    def _create_network_stats_panel(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create network statistics panel for system mode."""
        network_table = Table(show_header=False, box=None, padding=0, expand=True)
        network_table.add_column("Info", style="bold")

        try:
            # Network Status
            net_status = f"[{ColorScheme.INFO}]Network Status[/{ColorScheme.INFO}]\n"
            if metrics and metrics.network_latency is not None:
                latency_color = ColorScheme.get_performance_color(metrics.network_latency, 100, 50)
                net_status += f"Ping to 8.8.8.8: [{latency_color}]{metrics.network_latency:.1f} ms[/{latency_color}]"
            else:
                net_status += "Ping: checking..."

            network_table.add_row(net_status)

            # Network Interfaces
            net_stats = psutil.net_io_counters(pernic=True)
            interfaces_info = f"\n[{ColorScheme.ACCENT}]Network Interfaces[/{ColorScheme.ACCENT}]\n"

            active_interfaces = [
                (interface, stats) for interface, stats in list(net_stats.items())[:5]
                if stats.bytes_sent > 0 or stats.bytes_recv > 0
            ]

            for interface, stats in active_interfaces:
                sent_mb = stats.bytes_sent / (1024**2)
                recv_mb = stats.bytes_recv / (1024**2)
                interfaces_info += f"{interface}: Sent {sent_mb:.1f}MB, Recv {recv_mb:.1f}MB\n"

            network_table.add_row(interfaces_info.rstrip())

            # System Status
            if metrics:
                sys_status = f"\n[{ColorScheme.STATUS_WARN}]System Status[/{ColorScheme.STATUS_WARN}]\n"
                sys_status += f"Uptime: {metrics.uptime}"

                if metrics.temperature:
                    temp_color = ColorScheme.get_performance_color(metrics.temperature, 70, 50)
                    sys_status += f"\nTemperature: [{temp_color}]{metrics.temperature:.1f}°C[/{temp_color}]"

                if metrics.battery_percent:
                    batt_color = ColorScheme.get_performance_color(100 - metrics.battery_percent, 80, 50)
                    sys_status += f"\nBattery: [{batt_color}]{metrics.battery_percent:.1f}%[/{batt_color}]"

                network_table.add_row(sys_status)

        except Exception as e:
            return Panel(
                f"[{ColorScheme.STATUS_ERROR}]Error loading network stats: {str(e)}[/{ColorScheme.STATUS_ERROR}]",
                title="Network & Status",
                border_style=ColorScheme.STATUS_ERROR
            )

        return Panel(network_table, title="Network & Status")

    def _create_help_panel(self) -> Panel:
        """Create help panel with keyboard shortcuts."""
        help_text = f"Commands: Press [{ColorScheme.STATUS_ERROR}]'x'[/{ColorScheme.STATUS_ERROR}] or [{ColorScheme.STATUS_ERROR}]Ctrl+C[/{ColorScheme.STATUS_ERROR}] to quit"
        return Panel(help_text, title="Help", border_style=ColorScheme.INFO)