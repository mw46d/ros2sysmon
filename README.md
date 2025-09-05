# ros2sysmon - ROS2 System Monitor

A command-line system monitor for ROS2 that displays real-time system and ROS-specific metrics in a beautiful terminal dashboard.

![ros2sysmon dashboard](screenshot.png)

## Features

- **Real-time System Monitoring**: CPU, memory, disk usage with visual progress bars
- **Network Latency**: Continuous ping monitoring to detect connectivity issues  
- **Temperature Monitoring**: System thermal sensors (CPU, thermal zones)
- **System Alerts**: Configurable thresholds with color-coded warnings
- **Rich Terminal UI**: Beautiful full-screen dashboard using Rich library
- **ROS2 Integration**: Built as a proper ROS2 package with colcon build system
- **Extensible Architecture**: Easy to add new collectors and display panels

## Installation

### Prerequisites

- ROS2 (Humble, Iron, or Jazzy)
- Python 3.8+
- colcon build tools

### Dependencies

The following Python packages will be installed automatically:
- `rich>=12.0.0` - Terminal UI library
- `psutil>=5.8.0` - System metrics collection
- `pyyaml>=6.0` - Configuration file parsing

### Build Instructions

1. Clone or place the package in your ROS2 workspace:
```bash
cd ~/your_ros2_workspace/src
# Place ros2sysmon directory here
```

2. Build the package:
```bash
cd ~/your_ros2_workspace
colcon build --packages-select ros2sysmon
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Running ros2sysmon

### Basic Usage

Run the system monitor with default settings:
```bash
ros2 run ros2sysmon ros2sysmon
```

Or use the shorter alias:
```bash
ros2 run ros2sysmon run
```

### Command Line Options

- `--config PATH` - Specify custom configuration file
- `--refresh-rate SECONDS` - Set update interval (default: 1.0)
- `--no-color` - Disable colored output

### Examples

```bash
# Run with faster refresh rate
ros2 run ros2sysmon run --refresh-rate 0.5

# Use custom configuration
ros2 run ros2sysmon run --config /path/to/custom_config.yaml

# Slower updates for less system load
ros2 run ros2sysmon run --refresh-rate 2.0
```

### Exiting

Press `Ctrl+C` to exit cleanly.

## Metric Calculations

### CPU Usage (`cpu_percent`)
- **Method**: `psutil.cpu_percent(interval=0.1)`
- **Calculation**: Percentage of CPU time used across all cores during a 0.1-second sampling interval
- **Range**: 0-100% (can exceed 100% on multi-core systems when averaged)
- **Update Frequency**: Every refresh cycle (default 1 second)
- **Notes**: Brief 0.1s blocking call for accurate measurement

### Memory Usage (`memory_percent`)
- **Method**: `psutil.virtual_memory().percent`
- **Calculation**: `(total - available) / total * 100`
- **Components**: 
  - `total`: Total physical RAM
  - `available`: Memory available for new processes (includes buffers/cache)
- **Range**: 0-100%
- **Notes**: Uses "available" memory which accounts for Linux buffer/cache that can be reclaimed

### Disk Usage (`disk_percent`)
- **Method**: `psutil.disk_usage('/').percent`
- **Calculation**: `used / total * 100` for root filesystem
- **Target**: Root filesystem (`/`)
- **Range**: 0-100%
- **Notes**: Only monitors the root partition; does not include other mounted filesystems

### System Load Average (`load_average`)
- **Method**: `psutil.getloadavg()`
- **Values**: Returns tuple of (1min, 5min, 15min) load averages
- **Calculation**: Number of processes waiting for CPU or I/O, averaged over time periods
- **Display**: Shows 1-minute load average
- **Interpretation**: 
  - `< 1.0`: System not fully utilized
  - `= 1.0`: System fully utilized 
  - `> 1.0`: System overloaded (on single-core systems)

### Temperature (`temperature`)
- **Method**: `psutil.sensors_temperatures()`
- **Sources Checked** (in order):
  1. `coretemp` - Intel CPU temperature sensors
  2. `cpu_thermal` - ARM/generic CPU thermal zones
- **Calculation**: Current temperature from first available sensor
- **Units**: Degrees Celsius (°C)
- **Fallback**: `None` if no temperature sensors available
- **Notes**: Returns first sensor reading; may vary by hardware

### System Uptime (`uptime`)
- **Method**: `time.time() - psutil.boot_time()`
- **Calculation**: Current time minus system boot time
- **Format**: `HH:MM` (hours:minutes)
- **Precision**: Truncated to minutes for display
- **Source**: System boot timestamp from `/proc/stat` or equivalent

### Battery Level (`battery_percent`)
- **Method**: `psutil.sensors_battery()`
- **Calculation**: Current battery charge percentage
- **Range**: 0-100%
- **Availability**: Only on systems with battery (laptops, UPS)
- **Fallback**: `None` if no battery detected
- **Notes**: May not be available on desktop systems or VMs

### Network Latency (`network_latency`)
- **Method**: `subprocess.run(['ping', '-c', '1', '-W', '2', '8.8.8.8'])`
- **Target**: Google DNS (8.8.8.8)
- **Calculation**: Round-trip time from ping output parsing
- **Pattern**: Extracts `time=X.XXXms` from ping response
- **Units**: Milliseconds (ms)
- **Frequency**: Every 5 seconds (not every refresh cycle)
- **Timeout**: 2-second wait, 3-second total timeout
- **Fallback**: `None` if ping fails (network down, timeout, etc.)
- **Notes**: Uses system `ping` command; requires network connectivity

## Alert Thresholds

Configurable warning and error thresholds in `config/default_config.yaml`:

### System Thresholds
- **CPU**: Warn at 70%, Error at 85%
- **Memory**: Warn at 75%, Error at 90%  
- **Disk**: Warn at 80%, Error at 95%
- **Temperature**: Warn at 70°C, Error at 85°C
- **Network Latency**: Warn at 100ms, Error at 500ms

### Alert Generation
- Alerts are generated when metrics exceed thresholds
- **WARN** (yellow): Performance degradation expected
- **ERROR** (red): Critical resource exhaustion
- Alerts persist in bottom panel with timestamps
- Maximum 50 alerts stored, older alerts are rotated out

## Configuration

### Default Configuration File
Located at `config/default_config.yaml`:

```yaml
# Refresh and display settings
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

# Display preferences  
display:
  show_colors: true
  show_progress_bars: true
  time_format: "%H:%M:%S"
```

### Custom Configuration
Create a custom YAML file and use `--config` option to override defaults.

## Architecture

### Components
- **DataCollectionManager**: Coordinates multiple collector threads
- **SystemCollector**: Gathers CPU, memory, disk, temperature metrics
- **NetworkCollector**: Monitors network latency via ping
- **DisplayManager**: Rich-based terminal UI with live updates
- **SharedDataStore**: Thread-safe data exchange between collectors and display

### Threading Model
- Main thread: UI rendering and user input
- System collector thread: Updates every refresh cycle
- Network collector thread: Pings every 5 seconds
- All threads coordinate via SharedDataStore with locks

## Future Features

- **ROS2 Node Monitor**: Display running ROS2 nodes with resource usage
- **Topic Metrics**: Monitor ROS2 topic frequencies and bandwidth
- **Service Discovery**: Track available ROS2 services
- **Custom Collectors**: Plugin architecture for additional metrics
- **Export Functionality**: Save metrics to CSV/JSON files

## Contributing

This package follows ROS2 development practices:
- Use `colcon build` for building
- Follow ROS2 package structure
- Maintain compatibility with ROS2 LTS releases
- Keep functions under 50 lines (see `claude.md`)
- Use idiomatic Python with type hints

## License

MIT License - see LICENSE file for details.