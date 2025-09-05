ROS2 System Monitor (ros2top) - Implementation Specification
Project Overview
Create a command-line system monitor for ROS2 that displays real-time system and ROS-specific metrics in a dashboard format, similar to htop but specialized for robotics applications.
Technical Requirements
Dependencies
bash# System requirements
pip install rich psutil pyyaml

# ROS2 dependencies (assumed to be installed)
rclpy sensor_msgs geometry_msgs std_msgs nav_msgs tf2_msgs
Project Structure
ros2top/
├── src/
│   ├── __init__.py
│   ├── main.py              # Entry point and CLI argument parsing
│   ├── display.py           # Rich-based display management
│   ├── collectors/
│   │   ├── __init__.py
│   │   ├── system_collector.py    # System metrics (CPU, memory, etc.)
│   │   ├── ros_collector.py       # ROS2 nodes, topics, services
│   │   └── network_collector.py   # Network and connectivity
│   ├── models.py            # Data structures and classes
│   └── config.py            # Configuration management
├── config/
│   └── default_config.yaml  # Default thresholds and settings
├── setup.py
└── README.md
Core Data Structures
SystemMetrics Class
python@dataclass
class SystemMetrics:
    cpu_percent: float
    memory_percent: float
    disk_percent: float
    temperature: Optional[float]
    load_average: Tuple[float, float, float]  # 1min, 5min, 15min
    uptime: str
    battery_percent: Optional[float]
    network_latency: Optional[float]
    timestamp: datetime
ROSNodeInfo Class
python@dataclass
class ROSNodeInfo:
    name: str
    pid: int
    cpu_percent: float
    memory_mb: float
    status: str  # "OK", "WARN", "ERROR"
    uptime: str
    namespace: str
TopicMetrics Class
python@dataclass
class TopicMetrics:
    name: str
    frequency_hz: float
    target_frequency: float
    bandwidth_bps: int
    message_count: int
    msg_type: str
    status: str  # "OK", "SLOW", "IDLE", "ERROR"
    last_seen: datetime
SystemAlert Class
python@dataclass
class SystemAlert:
    level: str  # "INFO", "WARN", "ERROR"
    message: str
    timestamp: datetime
    category: str  # "SYSTEM", "ROS", "NETWORK"
Configuration System
default_config.yaml
yaml# Refresh and display settings
refresh_rate: 1.0  # seconds
max_alerts: 10
max_nodes_display: 15
max_topics_display: 10

# System thresholds
thresholds:
  cpu_warn: 70.0
  cpu_error: 85.0
  memory_warn: 75.0
  memory_error: 90.0
  disk_warn: 80.0
  disk_error: 95.0
  temperature_warn: 70.0
  temperature_error: 85.0
  network_latency_warn: 100  # ms
  network_latency_error: 500

# ROS2 specific settings
ros:
  critical_topics:
    - name: "/tf"
      target_frequency: 30.0
    - name: "/tf_static"
      target_frequency: 1.0
    - name: "/scan"
      target_frequency: 20.0
    - name: "/camera/image_raw"
      target_frequency: 10.0
    - name: "/cmd_vel"
      target_frequency: 1.0
  
  node_patterns:
    ignore:
      - "/_ros2cli_*"
      - "/launch_ros_*"

# Display preferences
display:
  show_colors: true
  show_progress_bars: true
  time_format: "%H:%M:%S"
Implementation Details
1. Data Collection Architecture
Threading Model:
pythonclass DataCollectionManager:
    def __init__(self, config):
        self.collectors = [
            SystemCollector(config),
            ROSCollector(config),
            NetworkCollector(config)
        ]
        self.shared_data = SharedDataStore()
        self.running = threading.Event()
    
    def start_collection(self):
        for collector in self.collectors:
            thread = threading.Thread(target=collector.collect_loop, 
                                     args=(self.shared_data, self.running))
            thread.daemon = True
            thread.start()
Shared Data Store:
pythonclass SharedDataStore:
    def __init__(self):
        self._lock = threading.Lock()
        self.system_metrics = None
        self.ros_nodes = []
        self.topic_metrics = []
        self.alerts = deque(maxlen=50)
    
    def update_system_metrics(self, metrics: SystemMetrics):
        with self._lock:
            self.system_metrics = metrics
    
    # Similar methods for other data types
2. System Collector Implementation
pythonclass SystemCollector:
    def __init__(self, config):
        self.config = config
        self.last_network_check = time.time()
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        while running.is_set():
            try:
                metrics = self._collect_system_metrics()
                shared_data.update_system_metrics(metrics)
                
                # Generate alerts based on thresholds
                alerts = self._check_thresholds(metrics)
                for alert in alerts:
                    shared_data.add_alert(alert)
                    
            except Exception as e:
                # Log error and continue
                pass
            
            time.sleep(self.config.refresh_rate)
    
    def _collect_system_metrics(self) -> SystemMetrics:
        return SystemMetrics(
            cpu_percent=psutil.cpu_percent(interval=0.1),
            memory_percent=psutil.virtual_memory().percent,
            disk_percent=psutil.disk_usage('/').percent,
            temperature=self._get_temperature(),
            load_average=psutil.getloadavg(),
            uptime=self._get_uptime(),
            battery_percent=self._get_battery(),
            network_latency=self._check_network_latency(),
            timestamp=datetime.now()
        )
    
    def _get_temperature(self) -> Optional[float]:
        # Implementation for temperature reading
        # Try multiple sources: thermal sensors, CPU sensors
        pass
    
    def _check_network_latency(self) -> Optional[float]:
        # Ping test to reliable host (8.8.8.8)
        # Only check every 5 seconds to avoid spam
        pass
3. ROS2 Collector Implementation
pythonclass ROSCollector:
    def __init__(self, config):
        self.config = config
        rclpy.init()
        self.node = rclpy.create_node('ros2top_monitor')
        self.topic_subscribers = {}
        self.topic_stats = {}
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        while running.is_set():
            try:
                # Collect node information
                nodes = self._get_ros_nodes()
                shared_data.update_ros_nodes(nodes)
                
                # Collect topic metrics
                topics = self._get_topic_metrics()
                shared_data.update_topic_metrics(topics)
                
                # Spin ROS node for callbacks
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
            except Exception as e:
                # Log and continue
                pass
            
            time.sleep(self.config.refresh_rate)
    
    def _get_ros_nodes(self) -> List[ROSNodeInfo]:
        # Use ros2 node list and cross-reference with psutil processes
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True)
        node_names = result.stdout.strip().split('\n')
        
        nodes = []
        for name in node_names:
            if self._should_ignore_node(name):
                continue
            
            # Find corresponding process
            process_info = self._find_node_process(name)
            if process_info:
                nodes.append(ROSNodeInfo(
                    name=name,
                    pid=process_info['pid'],
                    cpu_percent=process_info['cpu'],
                    memory_mb=process_info['memory'],
                    status=self._determine_node_status(name, process_info),
                    uptime=process_info['uptime'],
                    namespace=name.split('/')[1] if '/' in name else ''
                ))
        return nodes
    
    def _get_topic_metrics(self) -> List[TopicMetrics]:
        # Use ros2 topic list and hz commands
        # Maintain rolling averages for frequency calculation
        topics = []
        for topic_config in self.config.ros.critical_topics:
            metrics = self._measure_topic(topic_config['name'])
            if metrics:
                topics.append(metrics)
        return topics
    
    def _measure_topic(self, topic_name: str) -> Optional[TopicMetrics]:
        # Implementation to measure topic frequency and bandwidth
        # Use subscription-based measurement for accuracy
        pass
4. Display System Implementation
pythonclass DisplayManager:
    def __init__(self, config):
        self.config = config
        self.console = Console()
        self.layout = self._create_layout()
    
    def run_display(self, shared_data: SharedDataStore):
        with Live(self.layout, console=self.console, refresh_per_second=1/self.config.refresh_rate) as live:
            while True:
                try:
                    self._update_display(shared_data)
                    time.sleep(self.config.refresh_rate)
                except KeyboardInterrupt:
                    break
    
    def _create_layout(self) -> Layout:
        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="body"),
            Layout(name="alerts", size=5)
        )
        layout["body"].split_row(
            Layout(name="left"),
            Layout(name="right")
        )
        return layout
    
    def _update_display(self, shared_data: SharedDataStore):
        with shared_data._lock:
            # Update header with system overview
            self.layout["header"].update(self._create_system_overview(shared_data.system_metrics))
            
            # Update left panel with nodes
            self.layout["left"].update(self._create_nodes_table(shared_data.ros_nodes))
            
            # Update right panel with topics
            self.layout["right"].update(self._create_topics_table(shared_data.topic_metrics))
            
            # Update alerts
            self.layout["alerts"].update(self._create_alerts_panel(shared_data.alerts))
    
    def _create_system_overview(self, metrics: SystemMetrics) -> Panel:
        if not metrics:
            return Panel("Loading system metrics...")
        
        # Create progress bars and text
        cpu_bar = self._create_progress_bar(metrics.cpu_percent, 100, "CPU")
        memory_bar = self._create_progress_bar(metrics.memory_percent, 100, "Memory")
        
        content = f"""
{cpu_bar} {metrics.cpu_percent:.1f}%    {memory_bar} {metrics.memory_percent:.1f}%    Temp: {metrics.temperature or 'N/A'}°C
Load: {metrics.load_average[0]:.1f}    Uptime: {metrics.uptime}    Network: {metrics.network_latency or 'N/A'}ms
        """.strip()
        
        return Panel(content, title="System Resources", border_style="blue")
    
    def _create_nodes_table(self, nodes: List[ROSNodeInfo]) -> Table:
        table = Table(title="ROS2 Nodes")
        table.add_column("Name", style="cyan")
        table.add_column("CPU%", justify="right")
        table.add_column("Memory(MB)", justify="right")
        table.add_column("Status")
        table.add_column("Uptime")
        
        for node in nodes[:self.config.max_nodes_display]:
            status_style = {
                "OK": "green",
                "WARN": "yellow", 
                "ERROR": "red"
            }.get(node.status, "white")
            
            table.add_row(
                node.name,
                f"{node.cpu_percent:.1f}%",
                f"{node.memory_mb:.1f}",
                Text(node.status, style=status_style),
                node.uptime
            )
        
        return table
5. Main Entry Point
pythondef main():
    parser = argparse.ArgumentParser(description="ROS2 System Monitor")
    parser.add_argument("--config", default="config/default_config.yaml")
    parser.add_argument("--refresh-rate", type=float, default=1.0)
    parser.add_argument("--no-color", action="store_true")
    args = parser.parse_args()
    
    # Load configuration
    config = ConfigManager.load_config(args.config)
    config.refresh_rate = args.refresh_rate
    
    # Initialize components
    shared_data = SharedDataStore()
    collector_manager = DataCollectionManager(config)
    display_manager = DisplayManager(config)
    
    try:
        # Start background collection
        collector_manager.start_collection()
        
        # Run display (blocking)
        display_manager.run_display(shared_data)
        
    finally:
        # Cleanup
        collector_manager.stop_collection()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
Key Implementation Notes

Error Handling: All collectors should handle exceptions gracefully and continue operation
Performance: Use subprocess caching for ROS2 CLI calls, limit expensive operations
Threading Safety: All shared data access must be thread-safe
Resource Cleanup: Properly cleanup ROS2 nodes and threads on exit
Extensibility: Design collector interface to easily add new metrics
Testing: Include unit tests for data collection and display formatting

Package Installation
python# setup.py
from setuptools import setup, find_packages

setup(
    name="ros2top",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        "rich>=12.0.0",
        "psutil>=5.8.0",
        "pyyaml>=6.0"
    ],
    entry_points={
        "console_scripts": [
            "ros2top=src.main:main",
        ],
    },
    python_requires=">=3.8",
)
This specification provides enough detail for a programmer to implement the complete system while maintaining flexibility for implementation choices within the defined architecture.RetryPSi want them not to overdo it on error checking. Errors that come from bugs or badly formatted yml should just print a message and exit. Typically you over do it on error handling for a quick and dirty application.EditROS2 System Monitor (ros2top) - Implementation Specification (Simplified Error Handling)
Error Handling Philosophy
Keep it simple: Let the application crash gracefully on programming errors, bad configs, or missing dependencies. Only catch errors that are expected during normal operation (like temporary network issues or ROS nodes going offline).
Error Categories:

Let it crash: Bad YAML, missing files, import errors, configuration errors
Catch and continue: Network timeouts, missing ROS nodes, temporary sensor failures
Catch and exit cleanly: KeyboardInterrupt (Ctrl+C)

Simplified Implementation Details
1. Configuration Loading (Fail Fast)
pythondef load_config(config_path: str) -> Config:
    # No try/catch - let yaml.load() crash if file is bad
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # Let it crash if required fields are missing
    return Config(
        refresh_rate=config_data['refresh_rate'],
        thresholds=config_data['thresholds'],
        ros=config_data['ros']
    )
2. System Collector (Minimal Error Handling)
pythonclass SystemCollector:
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        while running.is_set():
            # Only catch errors that might happen during normal operation
            try:
                metrics = self._collect_system_metrics()
                shared_data.update_system_metrics(metrics)
            except (psutil.NoSuchProcess, PermissionError):
                # Expected errors - process died or permission issue
                # Just skip this iteration
                pass
            
            time.sleep(self.config.refresh_rate)
    
    def _collect_system_metrics(self) -> SystemMetrics:
        # No error handling - let psutil crashes bubble up if system is broken
        return SystemMetrics(
            cpu_percent=psutil.cpu_percent(interval=0.1),
            memory_percent=psutil.virtual_memory().percent,
            disk_percent=psutil.disk_usage('/').percent,
            temperature=self._get_temperature(),
            # ... etc
        )
    
    def _get_temperature(self) -> Optional[float]:
        # Try a few common sources, return None if all fail
        try:
            temps = psutil.sensors_temperatures()
            if 'coretemp' in temps:
                return temps['coretemp'][0].current
        except:
            pass
        return None
3. ROS2 Collector (Fail on Setup, Continue on Runtime)
pythonclass ROSCollector:
    def __init__(self, config):
        self.config = config
        # Let these crash if ROS2 isn't set up properly
        rclpy.init()
        self.node = rclpy.create_node('ros2top_monitor')
    
    def collect_loop(self, shared_data: SharedDataStore, running: threading.Event):
        while running.is_set():
            # Only catch expected runtime errors
            try:
                nodes = self._get_ros_nodes()
                shared_data.update_ros_nodes(nodes)
                
                topics = self._get_topic_metrics()
                shared_data.update_topic_metrics(topics)
                
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
            except subprocess.CalledProcessError:
                # ros2 command failed - probably no nodes running
                shared_data.update_ros_nodes([])
            except rclpy.exceptions.ROSInterruptException:
                # ROS shutdown
                break
            
            time.sleep(self.config.refresh_rate)
    
    def _get_ros_nodes(self) -> List[ROSNodeInfo]:
        # Let subprocess crash if ros2 command doesn't exist
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, check=True)
        
        node_names = result.stdout.strip().split('\n')
        nodes = []
        
        for name in node_names:
            if not name.strip():  # Skip empty lines
                continue
                
            # Try to get process info, skip if not found
            try:
                process_info = self._find_node_process(name)
                if process_info:
                    nodes.append(ROSNodeInfo(...))
            except psutil.NoSuchProcess:
                # Process died between listing and checking - skip it
                continue
                
        return nodes
4. Display System (Minimal Guards)
pythonclass DisplayManager:
    def run_display(self, shared_data: SharedDataStore):
        # Only catch Ctrl+C for clean exit
        try:
            with Live(self.layout, console=self.console, refresh_per_second=1/self.config.refresh_rate) as live:
                while True:
                    self._update_display(shared_data)
                    time.sleep(self.config.refresh_rate)
        except KeyboardInterrupt:
            print("\nExiting...")
    
    def _update_display(self, shared_data: SharedDataStore):
        # No error handling - let Rich crash if display is broken
        with shared_data._lock:
            self.layout["header"].update(self._create_system_overview(shared_data.system_metrics))
            self.layout["left"].update(self._create_nodes_table(shared_data.ros_nodes))
            self.layout["right"].update(self._create_topics_table(shared_data.topic_metrics))
            self.layout["alerts"].update(self._create_alerts_panel(shared_data.alerts))
    
    def _create_system_overview(self, metrics: SystemMetrics) -> Panel:
        # Handle the one expected case - no data yet
        if not metrics:
            return Panel("Loading system metrics...")
        
        # Otherwise, assume data is valid - let it crash if not
        content = f"CPU: {metrics.cpu_percent:.1f}%  Memory: {metrics.memory_percent:.1f}%"
        return Panel(content, title="System Resources")
5. Main Entry Point (Fail Fast)
pythondef main():
    parser = argparse.ArgumentParser(description="ROS2 System Monitor")
    parser.add_argument("--config", default="config/default_config.yaml")
    parser.add_argument("--refresh-rate", type=float, default=1.0)
    args = parser.parse_args()
    
    # No error handling - let it crash if config is bad
    config = ConfigManager.load_config(args.config)
    config.refresh_rate = args.refresh_rate
    
    # No error handling - let it crash if initialization fails
    shared_data = SharedDataStore()
    collector_manager = DataCollectionManager(config)
    display_manager = DisplayManager(config)
    
    # Start everything - let it crash if setup fails
    collector_manager.start_collection()
    
    # Only handle clean exit
    try:
        display_manager.run_display(shared_data)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Basic cleanup
        collector_manager.stop_collection()
        rclpy.shutdown()
6. Network Collector (Simple Timeout)
pythonclass NetworkCollector:
    def _check_network_latency(self) -> Optional[float]:
        try:
            # Simple ping with timeout
            result = subprocess.run(['ping', '-c', '1', '-W', '2', '8.8.8.8'], 
                                  capture_output=True, timeout=3)
            if result.returncode == 0:
                # Parse the ping time from output
                return self._parse_ping_time(result.stdout.decode())
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            # Network is down or slow - return None
            pass
        
        return None
What NOT to Handle
Don't catch these - let them crash:

ImportError - missing dependencies
yaml.YAMLError - bad config file format
FileNotFoundError - missing config file
KeyError - missing required config fields
AttributeError - programming bugs
TypeError - programming bugs
Initial rclpy.init() failures
Rich display setup failures

Simple rules:

If it's a programming error → crash
If it's a setup/config error → crash with message
If it's expected during runtime → catch and continue
If it's user interrupt → catch and exit cleanly

This approach makes the code much cleaner and easier to debug, while still handling the runtime issues that are expected in a monitoring application.RetryClaude can make mistakes. Please double-check responses.