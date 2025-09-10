"""Display management using Textual for beautiful terminal UI."""

import time
import signal
from datetime import datetime
from textual.app import App
from textual.containers import Vertical, Horizontal
from textual.widgets import Static, DataTable
from typing import List, Optional
from .shared_data import SharedDataStore
from .data_models import SystemAlert, SystemMetrics, ROSNodeInfo, TopicMetrics, TFFrameInfo
from .config import Config


class DisplayManager(App):
    """Manages Textual-based terminal display with live updates."""
    
    CSS = """
    #header {
        height: 7;
        background: black;
        color: white;
        text-align: left;
    }
    
    #left {
        height: auto;
        width: 30%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #nodes_table {
        height: auto;
        width: auto;
        min-width: 25;
        max-width: 40;
        background: #0a0a0a;
        color: white;
        margin: 0 1;
    }
    
    #nodes_table > .datatable--header {
        background: black;
        color: ansi_bright_cyan;
        text-style: bold;
    }
    
    #nodes_table > .datatable--odd-row {
        background: #111111;
    }
    
    #nodes_table > .datatable--even-row {
        background: black;
    }
    
    #middle {
        height: auto;
        width: 70%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #topics_table {
        height: auto;
        width: auto;
        min-width: 40;
        max-width: 60;
        background: #0a0a0a;
        color: white;
        margin: 0 1;
    }
    
    #processes_table {
        height: auto;
        width: auto;
        min-width: 50;
        background: #141414;
        color: white;
        margin: 0 1;
    }
    
    #topics_table > .datatable--header {
        background: black;
        color: ansi_bright_magenta;
        text-style: bold;
    }
    
    #topics_table > .datatable--odd-row {
        background: #111111;
    }
    
    #topics_table > .datatable--even-row {
        background: black;
    }
    
    #processes_table > .datatable--header {
        background: black;
        color: ansi_bright_yellow;
        text-style: bold;
    }
    
    #processes_table > .datatable--odd-row {
        background: #111111;
    }
    
    #processes_table > .datatable--even-row {
        background: black;
    }
    
    #middle_ros {
        height: auto;
        width: 70%;
        background: black;
        color: white;
        text-align: left;
    }
    
    #tf_table {
        height: auto;
        width: 1fr;
        background: #141414;
        color: white;
        margin: 0 1;
    }
    
    #tf_table > .datatable--header {
        background: black;
        color: ansi_bright_green;
        text-style: bold;
    }
    
    #tf_table > .datatable--odd-row {
        background: #111111;
    }
    
    #tf_table > .datatable--even-row {
        background: black;
    }
    
    #alerts {
        height: 10;
        background: black;
        color: white;
        text-align: left;
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
            yield Static("Loading system metrics...", id="header", markup = False)
            
            # Screen 1 container (default visible) - Horizontal layout for Topics and TF Frames
            with Horizontal(id="mode1_container", classes="body"):
                topics_table = DataTable(id="topics_table")
                topics_table.add_columns("Topic", "Type", "Hz", "Count")
                yield topics_table
                tf_table = DataTable(id="tf_table")
                tf_table.add_columns("Frame", "Parent", "Recent")
                yield tf_table
            
            # Screen 2 container (initially hidden) - Horizontal layout for Nodes and Processes
            with Horizontal(id="mode2_container", classes="body hidden"):
                nodes_table = DataTable(id="nodes_table")
                nodes_table.add_columns("Node Name")
                yield nodes_table
                processes_table = DataTable(id="processes_table")
                processes_table.add_columns("PID", "Process", "CPU%", "Mem%")
                yield processes_table
            
            yield Static("No alerts", id="alerts")
            yield Static(self._create_help_panel(), id="help")

    def run_display(self, shared_data: SharedDataStore, data_manager=None):
        """Run the main display with data polling loop."""
        def exit_handler(signum, frame):
            self.exit_requested = True

        signal.signal(signal.SIGTERM, exit_handler)


        # Store shared_data reference for the app
        self.shared_data = shared_data
        self.data_manager = data_manager

        try:
            self.run()
        finally:
            pass

    def on_mount(self):
        """Start the data update timer when app mounts."""
        self.set_interval(self.config.refresh_rate, self._update_display)

    def on_key(self, event):
        """Handle key press events."""
        if event.key == "x" or event.key == "q":
            self.exit_requested = True
            self.exit()
        elif event.key == "r":
            self._trigger_manual_refresh()
        elif event.key == "1":
            self.display_mode = "1"
            self._switch_to_mode_1_panes()
        elif event.key == "2":
            self.display_mode = "2"
            self._switch_to_mode_2_panes()
    
    def _trigger_manual_refresh(self):
        """Trigger immediate refresh of all collectors."""
        if self.data_manager:
            self._is_refreshing = True
            self._update_display()  # Update display to show "refreshing"
            self.data_manager.trigger_manual_refresh()
            # Clear refreshing indicator after a short delay
            self.set_timer(0.5, self._clear_refreshing)
    
    def _clear_refreshing(self):
        """Clear the refreshing indicator."""
        self._is_refreshing = False
        self._update_display()

    def _switch_to_mode_2_panes(self):
        """Switch to screen 2 panes (Nodes, Processes)."""
        # Show screen 2 container
        self.query_one("#mode2_container").remove_class("hidden")
        
        # Hide screen 1 container
        self.query_one("#mode1_container").add_class("hidden")

    def _switch_to_mode_1_panes(self):
        """Switch to screen 1 panes (Topics, TF Frames)."""
        # Hide screen 2 container
        self.query_one("#mode2_container").add_class("hidden")
        
        # Show screen 1 container
        self.query_one("#mode1_container").remove_class("hidden")

    def _update_ros_panels(self, nodes, topics, tf_frames):
        """Update the Screen 1 panels."""
        # Update topics table (left in Screen 1)
        self._update_topics_table(topics)
        
        # Update TF frames table (right in Screen 1)
        self._update_tf_table(tf_frames)

    def _update_display(self):
        """Update all display panels with current data."""
        if not hasattr(self, 'shared_data'):
            return
            
        try:
            metrics, nodes, topics, tf_frames, processes, alerts = self.shared_data.get_system_data()
            
            # Update each panel with new data
            self._update_header_panel(metrics)
            self._update_nodes_table(nodes)
            self._update_processes_table(processes)
            self._update_ros_panels(nodes, topics, tf_frames)
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

        # Build content with metrics in two columns
        refresh_indicator = " refreshing" if getattr(self, '_is_refreshing', False) else ""
        
        # Column 1 (left side)
        col1 = []
        col1.append(f"Time: {metrics.timestamp.strftime('%H:%M:%S')}{refresh_indicator}")
        
        cpu_bar = make_progress_bar(metrics.cpu_percent)
        mem_bar = make_progress_bar(metrics.memory_percent)
        disk_bar = make_progress_bar(metrics.disk_percent)
        
        col1.append(f"CPU:    {cpu_bar}")
        col1.append(f"Memory: {mem_bar}")  
        col1.append(f"Disk:   {disk_bar}")
        
        # Column 2 (right side)
        col2 = []
        from . import __version__
        col2.append(f"ROS2 SysMon v{__version__} - Mode: {self.display_mode.upper()}")
        col2.append(f"Load: {metrics.load_average[0]:.2f}")
        col2.append(f"Uptime: {metrics.uptime}")
        
        if metrics.temperature:
            col2.append(f"Temp: {metrics.temperature:.1f}°C")
        else:
            col2.append("")
            
        # Additional info for any remaining space
        extra_info = []
        if metrics.network_latency is not None:
            extra_info.append(f"Network: {metrics.network_latency:.1f}ms")
        else:
            extra_info.append("Network: checking...")
            
        if metrics.battery_percent:
            extra_info.append(f"Battery: {metrics.battery_percent:.1f}%")
            
        # Add refresh intervals
        intervals = self.data_manager.config.collection_intervals if self.data_manager else None
        if intervals:
            interval_parts = []
            if intervals.system_metrics > 0:
                interval_parts.append(f"sys:{intervals.system_metrics}s")
            else:
                interval_parts.append("sys:manual")
            
            if intervals.network_ping > 0:
                interval_parts.append(f"net:{intervals.network_ping}s")
            else:
                interval_parts.append("net:manual")
                
            if intervals.ros_discovery > 0:
                interval_parts.append(f"ros:{intervals.ros_discovery}s")
            else:
                interval_parts.append("ros:manual")
                
            
            extra_info.append(f"Intervals: {' | '.join(interval_parts)}")
        
        # Combine columns side by side
        lines = []
        max_rows = max(len(col1), len(col2))
        
        for i in range(max_rows):
            left = col1[i] if i < len(col1) else ""
            right = col2[i] if i < len(col2) else ""
            # Pad left column to ~45 characters for alignment
            lines.append(f"{left:<45} {right}")
        
        # Add extra info on separate lines
        for info in extra_info:
            lines.append(info)
        
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

    def _update_nodes_table(self, nodes):
        """Update the nodes DataTable with current ROS node data."""
        try:
            nodes_table = self.query_one("#nodes_table")
            
            # Clear existing rows
            nodes_table.clear()
            
            if not nodes:
                # Add a single row indicating no nodes
                nodes_table.add_row("No ROS nodes")
                return
            
            # Sort nodes alphabetically for consistent display
            sorted_nodes = sorted(nodes, key=lambda n: n.name.lower())
            
            # Add rows for each node
            for node in sorted_nodes:
                # Format node name (truncate if too long)
                node_name = node.name.strip()
                if len(node_name) > 40:
                    node_name = node_name[:37] + "..."
                
                nodes_table.add_row(node_name)
                
        except Exception as e:
            # Fallback: show error in table
            try:
                nodes_table = self.query_one("#nodes_table")
                nodes_table.clear()
                nodes_table.add_row(f"Error: {str(e)[:30]}")
            except:
                pass
    
    def _update_tf_table(self, tf_frames):
        """Update the TF frames DataTable with current transform data."""
        try:
            tf_table = self.query_one("#tf_table")
            
            # Clear existing rows
            tf_table.clear()
            
            if not tf_frames:
                # Add a single row indicating no TF frames
                tf_table.add_row("No TF frames", "N/A", "never")
                return
            
            # Sort frames: root frames first, then by frame name
            sorted_frames = sorted(tf_frames, key=lambda f: (not f.is_root, f.frame_id.lower()))
            
            # Add rows for each TF frame
            for frame in sorted_frames:
                # Format frame name (truncate if too long)
                frame_name = frame.frame_id
                if len(frame_name) > 25:
                    frame_name = frame_name[:22] + "..."
                
                # Format parent name
                parent_name = frame.parent_frame if frame.parent_frame else "ROOT"
                if len(parent_name) > 20:
                    parent_name = parent_name[:17] + "..."
                
                # Format timestamp
                if frame.most_recent_transform == 0.0:
                    recent_time = "never"
                else:
                    from datetime import datetime
                    dt = datetime.fromtimestamp(frame.most_recent_transform)
                    recent_time = dt.strftime("%H:%M:%S")
                
                # No visual indicators needed
                
                tf_table.add_row(
                    frame_name,
                    parent_name,
                    recent_time
                )
                
        except Exception as e:
            # Fallback: show error in table
            try:
                tf_table = self.query_one("#tf_table")
                tf_table.clear()
                tf_table.add_row(f"Error: {str(e)[:20]}", "N/A", "error")
            except:
                pass

    def _update_processes_table(self, processes):
        """Update the processes DataTable with current process data."""
        try:
            processes_table = self.query_one("#processes_table")
            
            # Clear existing rows
            processes_table.clear()
            
            # Use processes from shared data
            ros_processes = processes
            
            if not ros_processes:
                # Add a single row indicating no processes
                processes_table.add_row("N/A", "No ROS processes", "0.0", "0.0")
                return
            
            # Sort by CPU usage (highest first) and take top 15 processes
            top_processes = sorted(
                ros_processes, 
                key=lambda p: p['cpu_percent'], 
                reverse=True
            )[:15]
            
            # Add rows for each process
            for proc in top_processes:
                # Format process name (truncate if too long)
                proc_name = proc['name']
                if len(proc_name) > 20:
                    proc_name = proc_name[:17] + "..."
                
                # Format data
                pid_str = str(proc['pid'])
                cpu_str = f"{proc['cpu_percent']:.1f}"
                mem_str = f"{proc['memory_percent']:.1f}"
                
                processes_table.add_row(
                    pid_str,
                    proc_name,
                    cpu_str,
                    mem_str
                )
                
        except Exception as e:
            # Fallback: show error in table
            try:
                processes_table = self.query_one("#processes_table")
                processes_table.clear()
                processes_table.add_row("ERR", f"Error: {str(e)[:15]}", "0.0", "0.0")
            except:
                pass


    def _update_alerts_panel(self, alerts):
        """Update the alerts panel."""
        alerts_widget = self.query_one("#alerts")
        content = self._create_alerts_panel(alerts)
        alerts_widget.update(content)

    def _create_alerts_panel(self, alerts: List[SystemAlert]) -> str:
        """Create the alerts panel content."""
        if not alerts:
            return "No alerts"

        # Show last few alerts (fit within panel height - doubled from 6 to 12)
        recent_alerts = list(alerts)[-9:]
        alert_lines = []

        for alert in recent_alerts:
            # Format time and message
            time_str = alert.timestamp.strftime("%H:%M:%S")
            level_indicator = {
                "ERROR": "ERR", 
                "WARN": "WRN", 
                "INFO": "INF"
            }.get(alert.level, alert.level)
            
            # Create alert line with more reasonable length
            alert_line = f"[{level_indicator}] {time_str} - {alert.message}"
            
            # Truncate if too long (increased from 50 to 120 characters)
            if len(alert_line) > 120:
                alert_line = alert_line[:117] + "..."
                
            alert_lines.append(alert_line)
        
        return "\n".join(alert_lines)

    def _update_help_panel(self):
        """Update the help panel."""
        help_widget = self.query_one("#help")
        content = self._create_help_panel()
        help_widget.update(content)

    def _create_help_panel(self) -> str:
        """Create help panel content with keyboard shortcuts."""
        return "Mode: 1 or 2; Refresh: r, exit: x or q"

    def _update_topics_table(self, topics):
        """Update the topics DataTable with current topic data."""
        try:
            topics_table = self.query_one("#topics_table")
            
            # Clear existing rows
            topics_table.clear()
            
            if not topics:
                # Add a single row indicating no topics
                topics_table.add_row("No topics", "N/A", "0.0", "0")
                return
            
            # Sort topics by frequency (highest first)
            sorted_topics = sorted(topics, key=lambda t: t.frequency_hz, reverse=True)
            
            # Add rows for each topic
            for topic in sorted_topics:
                # Format topic name (truncate if too long)
                topic_name = topic.name
                if len(topic_name) > 45:
                    topic_name = topic_name[:42] + "..."
                
                # Format message type (show just the message name)
                msg_type = topic.msg_type.split('/')[-1] if '/' in topic.msg_type else topic.msg_type
                if len(msg_type) > 20:
                    msg_type = msg_type[:17] + "..."
                
                # Format frequency
                freq_str = f"{topic.frequency_hz:.2f}"
                
                topics_table.add_row(
                    topic_name,
                    msg_type,
                    freq_str,
                    str(topic.message_count)
                )
                
        except Exception as e:
            # Fallback: show error in table
            try:
                topics_table = self.query_one("#topics_table")
                topics_table.clear()
                topics_table.add_row("Error", "N/A", "0.0", "0")
            except:
                pass

