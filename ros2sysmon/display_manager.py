"""Display management using Textual for beautiful terminal UI."""

import time
import signal
from datetime import datetime
from textual.app import App
from textual.containers import Vertical, Horizontal
from textual.widgets import Static, DataTable
from textual.screen import ModalScreen
from typing import List, Optional
from .shared_data import SharedDataStore
from .data_models import SystemAlert, SystemMetrics, ROSNodeInfo, TopicMetrics, TFFrameInfo
from .config import Config


class HelpScreen(ModalScreen):
    """Modal screen to display help text."""

    CSS = """
    HelpScreen {
        align: center middle;
    }

    HelpScreen > Vertical {
        background: $surface;
        border: thick $primary;
        width: 60;
        height: auto;
        padding: 1;
    }

    #help_title {
        text-align: center;
        text-style: bold;
        color: $primary;
    }

    #help_content {
        margin: 1 0;
        text-align: center;
    }

    #help_footer {
        text-align: center;
        color: $text-muted;
        text-style: italic;
    }
    """

    def compose(self):
        with Vertical():
            yield Static("Help", id="help_title")
            yield Static("", id="help_content")
            yield Static("Press any key to close", id="help_footer")

    def on_mount(self):
        """Load help text when screen mounts."""
        try:
            import os
            help_file = os.path.join(os.path.dirname(__file__), 'help.txt')
            with open(help_file, 'r') as f:
                help_text = f.read().strip()
        except Exception:
            help_text = "1=topics 2=nodes h=help r=refresh x=exit"

        help_widget = self.query_one("#help_content")
        help_widget.update(help_text)

    def on_key(self, event):
        """Close help screen on any key press."""
        self.dismiss()


class DisplayManager(App):
    """Manages Textual-based terminal display with live updates."""
    
    CSS_PATH = "sysmon.tcss"

    def __init__(self, config: Config):
        """Initialize the display manager."""
        super().__init__()
        self.config = config
        self.exit_requested = False
        self.display_mode = "1"

    def compose(self):
        """Create the layout structure dynamically based on configuration."""
        with Vertical():
            yield Static("Loading system metrics...", id="header", markup = False)

            # Create screen containers dynamically
            yield from self._create_screen_containers()

            yield Static("No alerts", id="alerts")
            yield Static(self._create_help_panel(), id="help")

    def _create_screen_containers(self):
        """Create screen containers with panels based on configuration."""
        layout_config = self.config.display.panel_layout

        # Create screen 1 container
        screen_1_panels = layout_config.get_panels_for_screen(1)
        if screen_1_panels:
            with Horizontal(id="mode1_container", classes="body"):
                for panel_name in screen_1_panels:
                    yield self._create_panel_widget(panel_name)
        else:
            # Empty screen with placeholder
            with Horizontal(id="mode1_container", classes="body"):
                yield Static("Screen 1 - No panels configured", id="screen1_empty")

        # Create screen 2 container
        screen_2_panels = layout_config.get_panels_for_screen(2)
        if screen_2_panels:
            with Horizontal(id="mode2_container", classes="body hidden"):
                for panel_name in screen_2_panels:
                    yield self._create_panel_widget(panel_name)
        else:
            # Empty screen with placeholder
            with Horizontal(id="mode2_container", classes="body hidden"):
                yield Static("Screen 2 - No panels configured", id="screen2_empty")

    def _create_panel_widget(self, panel_name: str):
        """Create a panel widget based on panel name."""
        if panel_name == "topics":
            topics_table = DataTable(id="topics_table")
            topics_table.add_columns("Topic", "Type", "Hz", "Count")
            return topics_table
        elif panel_name == "tfs":
            tf_table = DataTable(id="tf_table")
            tf_table.add_columns("Frame", "Parent", "Recent")
            return tf_table
        elif panel_name == "nodes":
            nodes_table = DataTable(id="nodes_table")
            nodes_table.add_columns("Node Name")
            return nodes_table
        elif panel_name == "processes":
            processes_table = DataTable(id="processes_table")
            processes_table.add_columns("PID", "Process", "CPU%", "Mem%")
            return processes_table
        else:
            # Fallback for unknown panel names
            return Static(f"Unknown panel: {panel_name}", id=f"{panel_name}_unknown")

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
        elif event.key == "h":
            self._show_help()
    
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

    def _show_help(self):
        """Show help text in modal dialog."""
        self.push_screen(HelpScreen())

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
            nodes_table = self.query_one("#nodes_table", expect_type=DataTable)
        except Exception:
            # Nodes table not present (hidden panel)
            return

        try:
            
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
            tf_table = self.query_one("#tf_table", expect_type=DataTable)
        except Exception:
            # TF table not present (hidden panel)
            return

        try:
            
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
            processes_table = self.query_one("#processes_table", expect_type=DataTable)
        except Exception:
            # Processes table not present (hidden panel)
            return

        try:
            
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
        return "Mode: 1 or 2; Refresh: r; Help: h; Exit: x or q"

    def _filter_topics_for_display(self, topics):
        """Filter topics based on configuration display settings."""
        if not topics:
            return []

        # Get configured topics
        config_topics = self.config.ros.config_topics

        # Create lookup dictionaries for configured topics
        configured_topic_names = {topic.name for topic in config_topics if topic.name != "*"}
        configured_display_settings = {topic.name: topic.display for topic in config_topics if topic.name != "*"}

        # Find wildcard (*) configuration
        wildcard_config = next((topic for topic in config_topics if topic.name == "*"), None)
        wildcard_display = wildcard_config.display if wildcard_config else True

        filtered_topics = []

        for topic in topics:
            topic_name = topic.name

            if topic_name in configured_topic_names:
                # Topic is explicitly configured
                if configured_display_settings.get(topic_name, True):
                    filtered_topics.append(topic)
            else:
                # Topic not explicitly configured, use wildcard setting
                if wildcard_display:
                    filtered_topics.append(topic)

        return filtered_topics

    def _is_topic_hz_measurement_enabled(self, topic_name: str) -> bool:
        """Determine if Hz measurement is enabled for a specific topic."""
        # Get configured topics
        config_topics = self.config.ros.config_topics

        # Create lookup for measure_hz settings
        configured_topic_names = {topic.name for topic in config_topics if topic.name != "*"}
        measure_hz_settings = {topic.name: topic.measure_hz for topic in config_topics if topic.name != "*"}

        # Find wildcard (*) configuration
        wildcard_config = next((topic for topic in config_topics if topic.name == "*"), None)
        wildcard_measure_hz = wildcard_config.measure_hz if wildcard_config else False

        # Determine if Hz measurement is enabled for this topic
        if topic_name in configured_topic_names:
            return measure_hz_settings.get(topic_name, True)
        else:
            return wildcard_measure_hz

    def _format_topic_frequency(self, topic_name: str, frequency_hz: float) -> str:
        """Format frequency display based on whether Hz measurement is enabled."""
        if self._is_topic_hz_measurement_enabled(topic_name):
            return f"{frequency_hz:.2f}"
        else:
            return "--"

    def _format_topic_count(self, topic_name: str, message_count: int) -> str:
        """Format message count display based on whether Hz measurement is enabled."""
        if self._is_topic_hz_measurement_enabled(topic_name):
            return str(message_count)
        else:
            return "--"

    def _update_topics_table(self, topics):
        """Update the topics DataTable with current topic data."""
        try:
            topics_table = self.query_one("#topics_table", expect_type=DataTable)
        except Exception:
            # Topics table not present (hidden panel)
            return

        try:

            # Clear existing rows
            topics_table.clear()

            if not topics:
                # Add a single row indicating no topics
                topics_table.add_row("no topics yet...", "", "", "")
                return

            # Filter topics based on display configuration
            filtered_topics = self._filter_topics_for_display(topics)

            # Sort filtered topics by frequency (highest first)
            sorted_topics = sorted(filtered_topics, key=lambda t: t.frequency_hz, reverse=True)
            
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
                
                # Format frequency - check if Hz measurement is enabled for this topic
                freq_str = self._format_topic_frequency(topic.name, topic.frequency_hz)
                count_str = self._format_topic_count(topic.name, topic.message_count)

                topics_table.add_row(
                    topic_name,
                    msg_type,
                    freq_str,
                    count_str
                )
                
        except Exception as e:
            # Fallback: show error in table
            try:
                topics_table = self.query_one("#topics_table")
                topics_table.clear()
                topics_table.add_row("Error", "N/A", "0.0", "0")
            except:
                pass

