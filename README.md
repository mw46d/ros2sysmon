# ROS2 System Monitor

A terminal-based system monitor for ROS2 environments with real-time system metrics, ROS node/topic monitoring, and TF frame tracking.

## Current Status

* Version 0.2 - Active development with selective topic display features
* Verified to work on Ubuntu 24.04 with ROS2 Jazzy
* Requires separate installation of python library textual

![ROS2 System Monitor Screenshot](screenshot.png)
## Installation

### Prerequisites
- ROS2 (Humble, Iron, or Jazzy)
- Python 3.8+
- Python library textual

### Installing textual
You may find that doing any pip install gives an error `error: externally-managed-environment`. In our case it happened with `pip install textual`. You can work around this with `pip install textual --break-system-packages`. Note: This bypasses Python environment isolation - use with caution.

### Installing ros2sysmon

Like many other ROS2 apps:

```
cd ros2_ws/src
git clone https://github.com/pitosalas/ros2sysmon.git
```

### Build
```bash
cd ~/your_ros2_workspace
source /opt/ros/jazzy/setup.bash  # or your ROS2 distribution
colcon build --packages-select ros2sysmon
source install/setup.bash
```

## Usage

```bash
# Basic usage
ros2 run ros2sysmon ros2sysmon

# With options
ros2 run ros2sysmon ros2sysmon --config /path/to/config.yaml --refresh-rate 3.0
```

### Controls
- **`1`** - Topics + TF frames view
- **`2`** - ROS nodes + processes view  
- **`r`** - Manual refresh/collect data
- **`x`/`q`** - Exit

## Display

### Header Panel
System metrics with ASCII progress bars:
- CPU/Memory/Disk usage with visual bars
- Load average, uptime, temperature
- Network latency to 8.8.8.8

### Two Display Screens
1. **Screen 1**: ROS Topics (Hz, Count) + TF Frames
2. **Screen 2**: ROS Nodes + System Processes (filtered for ROS)

### Topic Display Features
- **Selective Display**: Configure which topics to show via `config_topics`
- **Hz Measurement Control**: Choose which topics to measure frequency for
- **Wildcard Support**: Use "*" to control behavior for unconfigured topics
- **Visual Indicators**: "--" shown for Hz/Count when measurement disabled

### Alerts Panel
System alerts with timestamps for threshold violations.

## Metric Calculations

### System Metrics

#### CPU Usage
- **Method**: `psutil.cpu_percent(interval=0.1)`
- **Calculation**: Average CPU utilization over 100ms sampling period
- **Range**: 0-100% (can exceed 100% on multi-core systems)
- **Display**: Progress bar with percentage, color-coded by thresholds

#### Memory Usage
- **Method**: `psutil.virtual_memory().percent`
- **Calculation**: `(total - available) / total * 100`
- **Includes**: RAM usage excluding buffers/cache
- **Display**: Progress bar with percentage and absolute values (used/total GB)

#### Disk Usage
- **Method**: `psutil.disk_usage('/').percent`
- **Target**: Root filesystem (`/`) usage
- **Calculation**: `used_space / total_space * 100`
- **Display**: Progress bar with percentage and absolute values

#### Temperature
- **Method**: `psutil.sensors_temperatures()`
- **Sources**: `coretemp` (Intel) or `cpu_thermal` (ARM/embedded)
- **Fallback**: Shows `--` if no temperature sensors available
- **Units**: Celsius (°C)

#### Load Average
- **Method**: `psutil.getloadavg()`
- **Values**: 1-minute, 5-minute, 15-minute averages
- **Meaning**: Average number of processes waiting for CPU/I/O
- **Interpretation**: Values > CPU core count indicate system stress

#### Uptime
- **Method**: `time.time() - psutil.boot_time()`
- **Calculation**: Current time minus system boot timestamp
- **Format**: `HH:MM` (hours:minutes since boot)

#### Battery Level
- **Method**: `psutil.sensors_battery()`
- **Availability**: Only on battery-powered systems
- **Display**: Percentage or `--` if no battery detected

#### Network Latency
- **Method**: `ping -c 1 -W 2 8.8.8.8` subprocess call
- **Target**: Google DNS (8.8.8.8) for internet connectivity
- **Timeout**: 2 second wait, 3 second total timeout
- **Parsing**: Extracts `time=X.XXXms` from ping output
- **Fallback**: Shows `--` if network unavailable or ping fails

### ROS Metrics

#### Node Discovery
- **Method**: `ros2 node list` CLI command with subprocess
- **Timeout**: 10 second timeout to prevent hanging
- **Filtering**: Excludes nodes matching `node_patterns.ignore` patterns
- **Information**: Node name, namespace, status based on process detection

#### Topic Discovery
- **Method**: `ros2 topic list -t` CLI command for topics with types
- **Timeout**: 5 second timeout
- **Processing**: Parses topic names and message types
- **Filtering**: Only configured topics shown based on `display: true`

#### Topic Hz Measurement
- **Subscription**: Creates ROS2 subscribers only for `measure_hz: true` topics
- **Collection Window**: Configurable duration (default 5.0s via `hz_collection_duration`)
- **Message Counting**: Timestamps collected during active measurement window
- **Calculation**: `message_count / collection_window_duration`
- **Precision**: 2 decimal places (e.g., "26.73 Hz")
- **Inactive Display**: Shows "--" when `measure_hz: false`
- **Error Handling**: Shows "0.00" if no messages received during window

#### Topic Message Count
- **Method**: Accumulated message count during Hz measurement window
- **Reset**: Counter resets at start of each measurement cycle
- **Display**: Integer count or "--" when measurement disabled
- **Relationship**: Used with time window to calculate Hz

#### TF Frame Discovery
- **Method**: `tf2_ros.Buffer.all_frames_as_yaml()`
- **Processing**: Parses YAML output to extract frame relationships
- **Information**: Frame ID, parent frame, transform timestamps
- **Root Detection**: Identifies frames without parents as root frames

#### Process Detection
- **Method**: `psutil.process_iter()` with keyword filtering
- **Keywords**: `["ros2", "rviz", "gazebo", "navigation", "moveit", "rqt", "robot_state", "joint_state", "launch", "python3", "map_server", "nav2", "slam"]`
- **Matching**: Checks process name and command line arguments
- **Metrics**: PID, name, CPU percentage (0.01s interval), memory percentage
- **Display**: Filtered list of ROS-related processes only

### Target Frequencies (for alerts)
- `/cmd_vel`, `/twist`: 10Hz
- `/odom`: 30Hz  
- `/scan`, lidar topics: 10Hz
- `/image*`, camera topics: 30Hz
- `/imu`: 100Hz
- `/tf`: 100Hz

## Configuration

Default config: `config/default_config.yaml`

### Complete YAML Configuration Structure
```yaml
# Display settings
refresh_rate: 5.0           # Display update rate (seconds)
max_alerts: 10              # Maximum alerts to display
max_nodes_display: 15       # Maximum ROS nodes to show
max_topics_display: 10      # Maximum topics to show

# Collection intervals (seconds)
collection_intervals:
  system_metrics: 10.0      # CPU, memory, disk, temperature
  network_ping: 10.0        # Network latency checks
  ros_discovery: 30.0       # ROS node/topic discovery
  hz_collection_duration: 5.0  # Hz measurement window duration

# System alert thresholds
thresholds:
  cpu_warn: 70.0           # CPU usage warning (%)
  cpu_error: 85.0          # CPU usage error (%)
  memory_warn: 75.0        # Memory usage warning (%)
  memory_error: 90.0       # Memory usage error (%)
  disk_warn: 80.0          # Disk usage warning (%)
  disk_error: 95.0         # Disk usage error (%)
  temperature_warn: 70.0   # Temperature warning (°C)
  temperature_error: 85.0  # Temperature error (°C)
  network_latency_warn: 100    # Network latency warning (ms)
  network_latency_error: 500   # Network latency error (ms)

# ROS2 specific configuration
ros:
  # Topic-specific settings with granular control
  config_topics:
    - name: "/tf"
      target_frequency: 30.0  # Expected Hz for alerts
      measure_hz: true        # Enable Hz measurement
      display: true           # Show in topic list
    - name: "/scan"
      target_frequency: 20.0
      measure_hz: true
      display: true
    - name: "/cmd_vel"
      target_frequency: 1.0
      measure_hz: false       # Disable Hz measurement
      display: true
    - name: "/map"
      measure_hz: true        # No target_frequency = no alerts
      display: true
    - name: "/odom"
      measure_hz: true
      display: true

  # Node filtering patterns
  node_patterns:
    ignore:
      - "/_ros2cli_*"         # Ignore CLI nodes
      - "/launch_ros_*"       # Ignore launch nodes

# Display preferences
display:
  show_colors: true           # Enable colored output
  show_progress_bars: true    # Show ASCII progress bars
  time_format: "%H:%M:%S"     # Timestamp format
  panel_layout:
    screen_1: ["topics", "tfs"]      # Screen 1 layout
    screen_2: ["nodes", "processes"] # Screen 2 layout
    hidden: []                       # Hidden panels
```

### Topic Configuration Details
- **name**: Topic name or "*" wildcard for unconfigured topics
- **target_frequency**: Expected Hz for alert generation (optional)
- **measure_hz**: Enable/disable Hz measurement (`true`/`false`)
- **display**: Show topic in display (`true`/`false`)
- Topics without `measure_hz: true` show "--" for Hz and count
- Use manual refresh (`r` key) to update Hz measurements

## Architecture

- **Textual** framework for terminal UI
- **Threading**: Separate collectors for system, network, and ROS data
- **SharedDataStore**: Thread-safe data exchange
- **Manual/Timed Collection**: Configurable collection intervals

## Dependencies

**ROS2**: `rclpy`, `tf2_ros`, `rosidl_runtime_py`  
**System**: `psutil`, `pyyaml`, `textual`  
**Standard**: `subprocess`, `threading`, `dataclasses`

## License

MIT
