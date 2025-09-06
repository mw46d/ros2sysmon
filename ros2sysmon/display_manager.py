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
from typing import List, Optional
from .shared_data import SharedDataStore
from .data_models import SystemAlert, SystemMetrics, ROSNodeInfo, TopicMetrics
from .config import Config


class DisplayManager:
    """Manages Rich-based terminal display with live updates."""
    
    def __init__(self, config: Config):
        """Initialize the display manager."""
        self.config = config
        self.console = Console()
        self.layout = self._create_layout()
        self.exit_requested = False
        self.display_mode = 'system'  # Default mode
    
    def run_display(self, shared_data: SharedDataStore):
        """Run the main display loop with Rich Live."""
        # Setup signal handlers for exit commands
        def exit_handler(signum, frame):
            self.exit_requested = True
            
        signal.signal(signal.SIGTERM, exit_handler)
        
        
        with Live(
            self.layout, 
            console=self.console, 
            refresh_per_second=1/self.config.refresh_rate,
            screen=True  # Full screen mode
        ) as live:
            try:
                while not self.exit_requested:
                    # Update display
                    self._update_display(shared_data)
                    
                    time.sleep(self.config.refresh_rate)
            except KeyboardInterrupt:
                pass
    
    def _create_layout(self) -> Layout:
        """Create the main layout structure."""
        layout = Layout()
        
        layout.split_column(
            Layout(name="header", size=7),  # Reduced since help moved to bottom
            Layout(name="body"),
            Layout(name="alerts", size=8),  # Alerts section
            Layout(name="help", size=1)     # Single row for help
        )
        
        layout["body"].split_row(
            Layout(name="left"),
            Layout(name="middle"), 
            Layout(name="right")
        )
        
        # Initialize with loading panels immediately
        self._initialize_panels(layout)
        
        return layout
    
    def _initialize_panels(self, layout: Layout):
        """Initialize all panels with loading messages immediately."""
        # Initialize header
        layout["header"].update(
            Panel("Loading system metrics...", title="Header", box=box.ROUNDED)
        )
        
        # Initialize body panes
        layout["left"].update(
            Panel("Left pane disabled", title="Left", box=box.ROUNDED)
        )
        layout["middle"].update(
            Panel("Middle pane disabled", title="Middle", box=box.ROUNDED)
        )
        layout["right"].update(
            Panel("Right pane disabled", title="Right", box=box.ROUNDED)
        )
        
        # Initialize alerts panel
        layout["alerts"].update(
            Panel("Loading alerts...", title="System Alerts", box=box.ROUNDED)
        )
        
        # Initialize help panel
        layout["help"].update(
            Text("Help disabled for testing")
        )
    
    def _update_display(self, shared_data: SharedDataStore):
        """Update all display panels with current data."""
        try:
            metrics, nodes, topics, alerts = shared_data.get_system_data()
            
            # Update header with system overview (disabled for testing)
            # self.layout["header"].update(
            #     self._create_system_overview(metrics)
            # )
            self.layout["header"].update(
                Panel("Header disabled for testing", title="Header", box=box.ROUNDED)
            )
            
            # Disable all body panes for testing
            # if self.display_mode == 'ros':
            #     # ROS Mode: Nodes | TF Frames | Topics
            #     self.layout["left"].update(
            #         self._create_nodes_panel(nodes)
            #     )
            #     self.layout["middle"].update(
            #         Text(
            #             "TF Frames\n(Coming soon)", 
            #             style="white on purple"
            #         )
            #     )
            #     self.layout["right"].update(
            #         self._create_topics_panel(topics)
            #     )
            # else:  # system mode
            #     # System Mode: Processes | CPU/Memory Details | Network Stats
            #     self.layout["left"].update(
            #         self._create_processes_panel()
            #     )
            #     self.layout["middle"].update(
            #         self._create_system_details_panel(metrics)
            #     )
            #     self.layout["right"].update(
            #         self._create_network_stats_panel(metrics)
            #     )
            
            # Show simple placeholders for body panes
            self.layout["left"].update(
                Panel("Left pane disabled", title="Left", box=box.ROUNDED)
            )
            self.layout["middle"].update(
                Panel("Middle pane disabled", title="Middle", box=box.ROUNDED)
            )
            self.layout["right"].update(
                Panel("Right pane disabled", title="Right", box=box.ROUNDED)
            )
            
            # Update alerts panel
            self.layout["alerts"].update(
                self._create_alerts_panel(alerts)
            )
            
            # Update help panel (disabled for testing)
            # self.layout["help"].update(
            #     self._create_help_panel()
            # )
            self.layout["help"].update(
                Text("Help disabled for testing")
            )
        except Exception as e:
            # Fallback - show error in header
            self.layout["header"].update(
                Panel(f"Error: {e}", title="Display Error", box=box.ROUNDED)
            )
    
    def _create_system_overview(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create the system overview panel with progress bars in two columns."""
        if not metrics:
            return Panel(
                "Loading system metrics...", 
                title="System Resources", 
                box=box.ROUNDED
            )
        
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
        
        # Add mode indicator only
        content_lines.append("")
        mode_text = f"Mode: {self.display_mode.upper()}"
        content_lines.append(mode_text)
        
        return Panel(
            "\n".join(content_lines),
            title=f"System Resources - {metrics.timestamp.strftime('%H:%M:%S')}",
            box=box.ROUNDED
        )
    
    def _create_system_overview_colored(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create the system overview panel with colored background."""
        if not metrics:
            return self._create_colored_pane("Loading system metrics...", "grey23")
        
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
        
        # Add mode indicator only
        content_lines.append("")
        mode_text = f"Mode: {self.display_mode.upper()}"
        content_lines.append(mode_text)
        
        content = "\n".join(content_lines)
        timestamp = metrics.timestamp.strftime('%H:%M:%S')
        full_content = f"System Resources - {timestamp}\n{content}"
        
        return self._create_colored_pane(full_content, "grey23")
    
    def _create_colored_pane(self, text: str, bg_color: str) -> Panel:
        """Create a pane with full background color fill."""
        return Panel(
            text,
            style=f"bright_white on {bg_color}",
            box=None,  # No border
            expand=True  # Fill entire space
        )
    
    def _create_nodes_panel_colored(self, nodes: List[ROSNodeInfo]) -> Panel:
        """Create the ROS nodes panel with colored background."""
        if not nodes:
            return self._create_colored_pane("ROS2 Nodes\nNo ROS nodes detected", "grey35")
        
        content = f"ROS2 Nodes ({len(nodes)} total)\n"
        for node in nodes[:10]:  # Show first 10 nodes
            content += f"{node.name} - {node.status}\n"
        
        return self._create_colored_pane(content, "grey35")
    
    def _create_topics_panel_colored(self, topics: List[TopicMetrics]) -> Panel:
        """Create the topics panel with colored background."""
        if not topics:
            return self._create_colored_pane("ROS2 Topics\nNo topics detected", "grey42")
        
        content = f"ROS2 Topics ({len(topics)} total)\n"
        for topic in topics[:10]:  # Show first 10 topics
            content += f"{topic.name} - {topic.frequency_hz:.1f}Hz\n"
        
        return self._create_colored_pane(content, "grey42")
    
    def _create_processes_panel_colored(self) -> Panel:
        """Create the processes panel with colored background."""
        return self._create_colored_pane("System Processes\n(Coming soon)", "grey35")
    
    def _create_system_details_panel_colored(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create the system details panel with colored background."""
        return self._create_colored_pane("System Details\n(Coming soon)", "grey39")
    
    def _create_network_stats_panel_colored(self, metrics: Optional[SystemMetrics]) -> Panel:
        """Create the network stats panel with colored background."""
        return self._create_colored_pane("Network Stats\n(Coming soon)", "grey42")
    
    def _create_help_panel_colored(self) -> Panel:
        """Create the help panel with colored background."""
        return self._create_colored_pane("Press Ctrl+C to exit", "grey50")
    
    def _create_alerts_panel(self, alerts: List[SystemAlert]) -> Panel:
        """Create the alerts panel."""
        if not alerts:
            return Panel(
                "No alerts",
                title="System Alerts", 
                box=box.ROUNDED
            )
        
        # Show last few alerts (more since we have more space)
        recent_alerts = list(alerts)[-6:]
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
        
        return Panel(
            "\n".join(alert_text),
            title=f"System Alerts ({len(alerts)} total)",
            box=box.ROUNDED
        )
    
    def _create_nodes_panel(self, nodes: List[ROSNodeInfo]) -> Panel:
        """Create the ROS nodes table panel."""
        if not nodes:
            return Panel(
                "No ROS nodes detected", 
                title="ROS2 Nodes", 
                box=box.ROUNDED
            )
        
        table = Table(show_header=True, header_style="bold blue", expand=True, show_lines=False, padding=0, box=None)
        table.add_column("Node", style="cyan", ratio=3)
        table.add_column("CPU%", justify="right", min_width=6)
        table.add_column("Mem(MB)", justify="right", min_width=8)
        table.add_column("Status", min_width=6)
        table.add_column("Uptime", min_width=8)
        
        # Sort nodes by CPU usage (highest first)
        sorted_nodes = sorted(nodes, key=lambda n: n.cpu_percent, reverse=True)
        
        for node in sorted_nodes[:10]:  # Show top 10 nodes
            # Use full node name since table can expand
            node_name = node.name
            
            # Color code status
            status_color = {
                "OK": "green",
                "WARN": "yellow", 
                "ERROR": "red"
            }.get(node.status, "white")
            
            table.add_row(
                node_name,
                f"{node.cpu_percent:.1f}",
                f"{node.memory_mb:.1f}",
                f"[{status_color}]{node.status}[/{status_color}]",
                node.uptime
            )
        
        return Panel(table, title=f"ROS2 Nodes ({len(nodes)} total)", box=box.ROUNDED)
    
    def _create_topics_panel(self, topics: List[TopicMetrics]) -> Panel:
        """Create the ROS topics table panel."""
        if not topics:
            return Panel(
                "No ROS topics detected", 
                title="ROS2 Topics", 
                box=box.ROUNDED
            )
        
        table = Table(show_header=True, header_style="bold green", expand=True, show_lines=False, padding=0, box=None)
        table.add_column("Topic", style="cyan", ratio=2)
        table.add_column("Hz", justify="right", min_width=6)
        table.add_column("Target", justify="right", min_width=6)
        table.add_column("Status", min_width=6)
        table.add_column("Type", ratio=1)
        
        # Sort topics by frequency (highest first)
        sorted_topics = sorted(topics, key=lambda t: t.frequency_hz, reverse=True)
        
        for topic in sorted_topics[:10]:  # Show top 10 topics
            # Use full names since table can expand
            topic_name = topic.name
            msg_type = topic.msg_type
            
            # Color code status
            status_color = {
                "OK": "green",
                "SLOW": "yellow",
                "IDLE": "red",
                "ERROR": "red"
            }.get(topic.status, "white")
            
            table.add_row(
                topic_name,
                f"{topic.frequency_hz:.1f}",
                f"{topic.target_frequency:.0f}",
                f"[{status_color}]{topic.status}[/{status_color}]",
                msg_type
            )
        
        return Panel(table, title=f"ROS2 Topics ({len(topics)} total)", box=box.ROUNDED)
    
    def _create_processes_panel(self) -> Text:
        """Create processes panel for system mode."""
        table = Table(show_header=True, header_style="bold white", expand=True, show_lines=False, padding=0, box=None)
        table.add_column("PID", justify="right", min_width=6)
        table.add_column("Process", ratio=2)
        table.add_column("CPU%", justify="right", min_width=6)
        table.add_column("Mem%", justify="right", min_width=6)
        table.add_column("Status", min_width=6)
        
        try:
            # Get ROS-related processes only
            ros_processes = []
            ros_keywords = ['ros2', 'rviz', 'gazebo', 'navigation', 'moveit', 'rqt', 'robot_state_publisher', 'joint_state_publisher']
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'cpu_percent', 'memory_percent', 'status']):
                try:
                    proc_info = proc.info
                    proc_name = proc_info['name'].lower()
                    cmdline = ' '.join(proc_info['cmdline']).lower() if proc_info['cmdline'] else ''
                    
                    # Check if process is ROS-related
                    is_ros_process = any(keyword in proc_name or keyword in cmdline for keyword in ros_keywords)
                    
                    if is_ros_process:
                        ros_processes.append(proc_info)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            
            # Sort by CPU usage and take top 12
            top_processes = sorted(ros_processes, key=lambda p: p['cpu_percent'] or 0, reverse=True)[:12]
            
            for proc in top_processes:
                cpu_pct = proc['cpu_percent'] or 0.0
                mem_pct = proc['memory_percent'] or 0.0
                
                # Color code by CPU usage
                cpu_color = "red" if cpu_pct > 50 else "yellow" if cpu_pct > 10 else "green"
                status_color = "green" if proc['status'] == 'running' else "yellow"
                
                table.add_row(
                    str(proc['pid']),
                    proc['name'][:30],  # Truncate long process names
                    f"[{cpu_color}]{cpu_pct:.1f}[/{cpu_color}]",
                    f"{mem_pct:.1f}",
                    f"[{status_color}]{proc['status']}[/{status_color}]"
                )
        except Exception:
            return Text(
                "ROS Processes - Error loading ROS processes", 
                style="bold white on red"
            )
        
        # Convert table to text with header
        from io import StringIO
        table_str = StringIO()
        self.console.print(table, file=table_str, end="")
        table_content = table_str.getvalue()
        
        header = "ROS Processes (Top CPU)"
        full_content = header + "\n" + table_content
        
        return Text(
            full_content,
            style="white on dark_magenta"
        )
    
    def _create_system_details_panel(self, metrics: Optional[SystemMetrics]) -> Text:
        """Create system details panel for system mode."""
        if not metrics:
            return Text(
                "System Details - Loading system details...", 
                style="bold white on dark_cyan"
            )
        
        content = []
        
        # CPU details
        content.append("[bold cyan]CPU Information[/bold cyan]")
        try:
            cpu_count = psutil.cpu_count(logical=True)
            cpu_freq = psutil.cpu_freq()
            load_1, load_5, load_15 = metrics.load_average
            
            content.append(f"Cores: {cpu_count} logical")
            if cpu_freq:
                content.append(f"Frequency: {cpu_freq.current:.0f} MHz")
            content.append(f"Load Avg: {load_1:.2f}, {load_5:.2f}, {load_15:.2f}")
            content.append("")
        except:
            content.append("CPU info unavailable")
            content.append("")
        
        # Memory details
        content.append("[bold green]Memory Information[/bold green]")
        try:
            vm = psutil.virtual_memory()
            swap = psutil.swap_memory()
            
            content.append(f"Total: {vm.total / (1024**3):.1f} GB")
            content.append(f"Available: {vm.available / (1024**3):.1f} GB")
            content.append(f"Used: {vm.used / (1024**3):.1f} GB ({vm.percent:.1f}%)")
            content.append(f"Swap: {swap.used / (1024**3):.1f} GB / {swap.total / (1024**3):.1f} GB")
            content.append("")
        except:
            content.append("Memory info unavailable")
            content.append("")
        
        # Disk details
        content.append("[bold magenta]Disk Information[/bold magenta]")
        try:
            disk = psutil.disk_usage('/')
            content.append(f"Total: {disk.total / (1024**3):.1f} GB")
            content.append(f"Used: {disk.used / (1024**3):.1f} GB ({metrics.disk_percent:.1f}%)")
            content.append(f"Free: {disk.free / (1024**3):.1f} GB")
        except:
            content.append("Disk info unavailable")
        
        # Add header and format content
        header = "System Details"
        full_content = header + "\n" + "\n".join(content)
        
        return Text(
            full_content, 
            style="white on dark_cyan"
        )
    
    def _create_network_stats_panel(self, metrics: Optional[SystemMetrics]) -> Text:
        """Create network statistics panel for system mode."""
        content = []
        
        # Network latency
        content.append("[bold blue]Network Status[/bold blue]")
        if metrics and metrics.network_latency is not None:
            latency_color = "red" if metrics.network_latency > 100 else "yellow" if metrics.network_latency > 50 else "green"
            content.append(f"Ping to 8.8.8.8: [{latency_color}]{metrics.network_latency:.1f} ms[/{latency_color}]")
        else:
            content.append("Ping: checking...")
        content.append("")
        
        # Network interfaces
        content.append("[bold cyan]Network Interfaces[/bold cyan]")
        try:
            net_stats = psutil.net_io_counters(pernic=True)
            for interface, stats in list(net_stats.items())[:5]:  # Show first 5 interfaces
                if stats.bytes_sent > 0 or stats.bytes_recv > 0:  # Only show active interfaces
                    sent_mb = stats.bytes_sent / (1024**2)
                    recv_mb = stats.bytes_recv / (1024**2)
                    content.append(f"{interface}:")
                    content.append(f"  Sent: {sent_mb:.1f} MB")
                    content.append(f"  Recv: {recv_mb:.1f} MB")
                    content.append("")
        except:
            content.append("Interface info unavailable")
            content.append("")
        
        # System uptime
        content.append("[bold yellow]System Status[/bold yellow]")
        if metrics:
            content.append(f"Uptime: {metrics.uptime}")
            if metrics.temperature:
                temp_color = "red" if metrics.temperature > 70 else "yellow" if metrics.temperature > 50 else "green"
                content.append(f"Temperature: [{temp_color}]{metrics.temperature:.1f}°C[/{temp_color}]")
            if metrics.battery_percent:
                batt_color = "red" if metrics.battery_percent < 20 else "yellow" if metrics.battery_percent < 50 else "green"
                content.append(f"Battery: [{batt_color}]{metrics.battery_percent:.1f}%[/{batt_color}]")
        
        # Add header and format content
        header = "Network & Status"
        full_content = header + "\n" + "\n".join(content)
        
        return Text(
            full_content, 
            style="white on dark_yellow"
        )
    
    def _create_help_panel(self) -> Panel:
        """Create help panel with keyboard shortcuts."""
        help_text = "Commands: Press Ctrl+C to quit"
        return Panel(
            help_text, 
            title="Help", 
            box=box.ROUNDED
        )