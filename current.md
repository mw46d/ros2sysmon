# ROS2 System Monitor - Current State

## Overview
A Python-based system monitoring application for ROS2 environments that provides real-time display of system metrics, ROS nodes, topics, TF frames, and processes using a Textual-based terminal UI.

## Architecture

### Core Components
- **DisplayManager** (`display_manager.py`) - Textual-based terminal UI with two display modes
- **SystemCollector** (`collectors/system_collector.py`) - Collects system metrics and ROS processes
- **NetworkCollector** (`collectors/network_collector.py`) - Network latency monitoring
- **ROSCollector** (`collectors/ros_collector.py`) - ROS2 node/topic/TF discovery and Hz collection
- **SharedDataStore** (`shared_data.py`) - Thread-safe data sharing between collectors and display
- **DataManager** - Coordinates all collectors and manages data flow

### Data Models
- **SystemMetrics** - CPU, memory, disk, temperature, network latency, uptime
- **ROSNodeInfo** - ROS node information
- **TopicMetrics** - Topic names, types, frequencies
- **TFFrameInfo** - Transform frame hierarchy and timestamps
- **SystemAlert** - Alert messages with timestamps and severity levels

## Display Interface

### Two Display Screens
1. **Screen 1** - Horizontal layout: Topics (left) | TF Frames (right)
2. **Screen 2** - Horizontal layout: Nodes (left) | Processes (right)

### UI Elements
- **Header Panel** - System metrics with progress bars, load average, uptime, temperature
- **Data Tables** - Color-coded headers for visual separation:
  - Nodes: `ansi_bright_cyan`
  - Topics: `ansi_bright_magenta`
  - Processes: `ansi_bright_yellow`
  - TF Frames: `ansi_bright_green`
- **Alerts Panel** - System alerts (truncation limit: 120 characters)
- **Help Panel** - Keyboard shortcuts

### Keyboard Controls
- `1` / `2` - Switch display screens
- `r` - Manual refresh of all collectors
- `x` / `q` - Exit application

## Collection System

### Collection Intervals (from config)
- **system_metrics**: 5.0s - System metrics and ROS processes
- **network_ping**: 0.0s (manual mode) - Network latency
- **ros_discovery**: 0.0s (manual mode) - ROS node/topic discovery
- **hz_collection_duration**: 3.0s - Duration for Hz measurement windows

### Threading Architecture
- **Main Thread** - Display refresh (5s intervals)
- **SystemCollector Thread** - System metrics + ROS process enumeration
- **NetworkCollector Thread** - Network ping (manual mode, 0.1s sleep loops)
- **ROSCollector Thread** - ROS discovery and Hz collection (manual mode, 0.1s sleep loops)

## Recent Bug Fixes & Optimizations

### Hz Collection Bug Fix
**Problem**: Topic Hz rates showed 0 for all topics except the first one due to Python lambda closure bug in `_setup_topic_subscribers()`.

**Solution**: 
- Fixed lambda closure in `ros_collector.py:251` by using proper closure with `make_callback()` function
- Each topic callback now correctly captures its own topic name
- All topics now show accurate Hz rates during collection windows

**Benefits**:
- Accurate frequency measurement for all ROS topics
- Proper topic monitoring and alerts

### Configuration Cleanup
**Problem**: Obsolete configuration settings causing AttributeError exceptions.

**Solution**:
- Removed unused `ros_callbacks` setting from all config files and data structures
- Removed obsolete `test_windowed_config.yaml` file
- Updated display manager to remove references to removed settings

**Benefits**:
- Cleaner configuration structure
- Eliminated runtime errors from missing attributes

### UI Terminology Update
- Changed "Mode 1/2" to "Screen 1/2" throughout the interface
- Removed visual icons from TF frame displays for cleaner appearance

## Previous Optimizations

### Process Collection Refactoring
**Problem**: Process enumeration (`_get_ros_processes()`) was running on display thread every 5 seconds, not governed by collection intervals.

**Solution**: 
- Moved process collection from `DisplayManager` to `SystemCollector`
- Added process storage to `SharedDataStore` with `update_processes()` method
- Updated `get_system_data()` tuple to include processes
- Process collection now respects `system_metrics` interval (5.0s)

**Benefits**:
- Reduced CPU overhead on display thread
- Proper interval governance for expensive `psutil.process_iter()` operations
- Better separation of concerns (data collection vs display)

### Visual Improvements
- Removed UI borders to reduce visual clutter
- Added subtle background color variations for section separation
- Implemented color-coded table headers with bright ANSI colors
- Increased alert message truncation limit from 50 to 120 characters

## Performance Characteristics

### Idle CPU/Memory Usage
- **Base overhead**: 1-3% CPU from Textual event loop
- **Periodic spikes**: 5-15% CPU every 5 seconds during:
  - System metrics collection
  - ROS process enumeration across all system processes
  - UI refresh and data structure updates
- **Memory usage**: ~15-25MB (Python + Textual + ROS2 context + data structures)

### Resource-Intensive Operations
1. `psutil.process_iter()` - Scans all system processes for ROS keywords
2. `proc.cpu_percent(interval=0.01)` - 10ms blocking call per ROS process
3. Complex UI rendering with multiple DataTables and CSS styling
4. Thread-safe data copying in SharedDataStore

## Configuration Files
- `collection_config.py` - Collection interval settings
- Various YAML config files for different deployment scenarios

## Alternative Implementations
- `display_manager_claude.py` - Rich-based display manager (alternative to Textual)

## Current Git Status
- Working directory has modifications to display_manager.py and collectors
- Several config files deleted/modified
- Archive directory and cache files present

## Key Technical Decisions
- Textual framework chosen for terminal UI over Rich
- Thread-based architecture for concurrent data collection
- Shared data store pattern for thread-safe communication
- Manual refresh mode for ROS collectors to reduce overhead
- Windowed Hz collection for accurate frequency measurement