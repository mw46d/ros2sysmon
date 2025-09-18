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
You may find that `pip install textual` gives an error `error: externally-managed-environment`. You can work around this with `pip install textual --break-system-packages`. Note: This bypasses Python environment isolation - use with caution.

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
- **CPU**: `psutil.cpu_percent(interval=0.1)` - 100ms sampling
- **Memory**: `psutil.virtual_memory().percent` - available vs total
- **Disk**: `psutil.disk_usage('/').percent` - root filesystem usage
- **Temperature**: `psutil.sensors_temperatures()` - from `coretemp`/`cpu_thermal`
- **Load Average**: `psutil.getloadavg()[0]` - 1-minute load
- **Uptime**: `time.time() - psutil.boot_time()` formatted as HH:MM
- **Network Latency**: `ping -c 1 -W 2 8.8.8.8` parsed from output

### ROS Metrics
- **Nodes**: Discovered via `ros2 node list` (10s timeout)
- **Topics**: Discovered via `ros2 topic list -t` (5s timeout)
- **Topic Hz**:
  - Creates ROS2 subscribers only for topics with `measure_hz: true`
  - Collects message timestamps during windowed collection (default 3s)
  - Calculates frequency as: `message_count / collection_window_duration`
  - Shows 2 decimal precision or "--" when measurement disabled
- **TF Frames**: Retrieved via `tf2_ros.Buffer.all_frames_as_yaml()`
- **Processes**: Filters system processes for ROS-related keywords

### Target Frequencies (for alerts)
- `/cmd_vel`, `/twist`: 10Hz
- `/odom`: 30Hz  
- `/scan`, lidar topics: 10Hz
- `/image*`, camera topics: 30Hz
- `/imu`: 100Hz
- `/tf`: 100Hz

## Configuration

Default config: `share/ros2sysmon/config/default_config.yaml`

### Topic Configuration
Topics are now configured via `config_topics` with granular control:
```yaml
ros:
  config_topics:
    - name: "/tf"
      target_frequency: 30.0
      measure_hz: true
      display: true
    - name: "/cmd_vel"
      target_frequency: 1.0
      measure_hz: false
      display: true
    - name: "*"           # Wildcard for all other topics
      measure_hz: false
      display: false
```

### Collection Intervals
- **system_metrics**: 5.0s - CPU, memory, disk, temperature
- **network_ping**: 5.0s - Network latency checks
- **ros_discovery**: 10.0s - Node/topic discovery
- **hz_collection_duration**: 3.0s - Hz measurement window

### Alert Thresholds
- **CPU**: Warn 70%, Error 85%
- **Memory**: Warn 75%, Error 90%
- **Disk**: Warn 80%, Error 95%  
- **Temperature**: Warn 70°C, Error 85°C
- **Network**: Warn 100ms, Error 500ms

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
