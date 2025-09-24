# ROS2 System Monitor - Current State

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Overview
A Python-based system monitoring application for ROS2 environments that provides real-time display of system metrics, ROS nodes, topics, TF frames, and processes using a Textual-based terminal UI.

## Recent Debug Session Findings

### Core Issue Discovered: Topic Subscription Mismatch
**Problem**: `/odom` and `/map` topics appear in display but show 0Hz despite being active
- **ROS node discovery** only finds 4 topics via `get_topic_names_and_types()`
- **CLI discovery** finds 70 topics via `ros2 topic list -t`
- **Only `/tf` gets subscribed** for Hz measurement
- **`/odom` and `/map` are never subscribed** despite having `measure_hz: true`

### Evidence from Debug Log Analysis:
- `/odom` actually publishes at ~26Hz (`ros2 topic hz /odom` confirms)
- `/tf` shows incorrect 90Hz (should be ~30Hz)
- Map server processes exist but weren't being detected in process list
- Manual refresh with 'r' key sometimes ignored due to timing windows

### Key Bug Locations:
1. **`ros_collector.py:244`** - `get_topic_names_and_types()` doesn't see all topics
2. **`ros_collector.py:343`** - Hz calculation may have message counting issues
3. **`display_manager.py:636`** - Topics filtered properly but Hz data missing

## Recent Improvements Made

### Process Detection Enhancement
- Added `"map_server"`, `"nav2"`, `"slam"` to ROS keywords list
- Removed 15-process limit - now shows all ROS processes
- Map server instances now properly detected and displayed

### UI/UX Improvements
- **Converted alerts panel to scrollable RichLog widget**
  - Shows last 50 alerts (up from 9)
  - Rich text formatting with colored levels (ERROR=red, WARN=yellow, INFO=blue)
  - Extended message length to 200 characters
  - Native scrolling support

### Code Quality Cleanup
- **Removed all print statements and debug logging** following claude.md rules
- **Fixed bare Exception blocks** - made specific or fail gracefully
- **Eliminated debug file pollution** - no more `/tmp/ros2sysmon_debug.log` writes
- **All error messages now route through alert system**

## Architecture Summary

### Core Components
- **DisplayManager** - Textual-based terminal UI with RichLog alerts
- **SystemCollector** - System metrics + ROS process enumeration
- **ROSCollector** - ROS2 discovery + Hz measurement (problematic)
- **NetworkCollector** - Network latency monitoring
- **SharedDataStore** - Thread-safe data sharing

### Display Interface
- **Screen 1**: Topics (left) | TF Frames (right)
- **Screen 2**: Nodes (left) | Processes (right)
- **Scrollable Alerts Panel**: Error/warning/info messages with color coding
- **Header Panel**: System metrics with progress bars

### Collection Intervals
- **system_metrics**: 5.0s
- **ros_discovery**: 30.0s (manual mode - press 'r')
- **hz_collection_duration**: 5.0s measurement windows

## Loose Ends & Priority Issues

### HIGH PRIORITY: Fix Topic Hz Measurement
**ROOT CAUSE**: ROS node discovery vs CLI discovery mismatch
- ROS node only sees 4 topics, CLI sees 70 topics
- Need to use CLI topic list for subscription setup, not just `get_topic_names_and_types()`
- `/odom` and `/map` never get subscribed despite config having `measure_hz: true`

### Manual Refresh Timing Issues
- 'r' key sometimes ignored due to 0.1s polling window
- Collection might be blocked during 5s Hz measurement windows
- Need more responsive refresh mechanism

### Code Architecture Violations
Following claude.md rules, several violations need addressing:
- **`ros_collector.py` is 610 lines** (should be ≤300)
- **Methods too long** (some >50 lines)
- **Debug code duplication** (mostly cleaned up)
- **File needs splitting** into smaller focused classes

### Hz Calculation Accuracy
- `/tf` shows 90Hz but actual is ~30Hz
- Possible message counting issues in windowed collection
- May be counting total messages across all topics instead of per-topic

## Next Steps Todo List

1. **[HIGH] Fix topic subscription discovery**
   - Modify `_setup_topic_subscribers()` to use CLI topic list
   - Ensure `/odom` and `/map` get proper subscriptions
   - Verify Hz measurement works for all configured topics

2. **[MEDIUM] Improve manual refresh responsiveness**
   - Reduce polling interval or use event-based triggering
   - Add immediate feedback when 'r' is pressed
   - Consider interrupting running collections for immediate refresh

3. **[MEDIUM] Refactor ros_collector.py**
   - Split into multiple files following claude.md (≤300 lines each)
   - Break large methods into smaller focused functions (≤50 lines)
   - Extract Hz calculation logic into separate class

4. **[LOW] Optimize performance**
   - Review message counting accuracy in Hz calculation
   - Consider async/await instead of threading where possible
   - Reduce CPU overhead during collection windows

## Configuration Status
- All config files cleaned up (removed obsolete `ros_callbacks` setting)
- Process keywords enhanced for better detection
- Display filtering working correctly
- Alert system fully functional with RichLog scrolling

## Technical Debt
- Large files violating 300-line limit (ros_collector.py)
- Some methods violating 50-line limit
- Threading architecture could benefit from async/await
- Error handling could be more specific in some areas

The system is functional but has the core Hz measurement issue that prevents proper topic frequency monitoring.