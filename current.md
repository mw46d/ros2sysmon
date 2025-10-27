# ROS2 System Monitor - Current State

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Overview
A Python-based system monitoring application for ROS2 environments that provides real-time display of system metrics, ROS nodes, topics, TF frames, and processes using a Textual-based terminal UI.

# ROS2 TF Timing Issues - Troubleshooting Context

**Date:** October 27, 2025  
**Platform:** Physical robot running ROS2  
**Issue:** Navigation stack dropping laser scan messages due to transform timing problems

---

## The Problem

The robot's navigation stack (Nav2) is experiencing severe message dropping with two types of errors:

1. **Transform timing errors** (most common):
   ```
   Message Filter dropping message: frame 'laser' at time X 
   for reason 'the timestamp on the message is earlier than all the data in the transform cache'
   ```

2. **Queue overflow errors** (SLAM specific):
   ```
   Message Filter dropping message: frame 'laser' at time X 
   for reason 'discarding message because the queue is full'
   ```

**Affected nodes:**
- `slam_toolbox` (SLAM/mapping)
- `planner_server` (path planning / global costmap)
- `controller_server` (robot control / local costmap)

---

## What We've Discovered

### System Architecture

**TF Publishers (3 nodes):**
1. `ekf_filter_node` - Extended Kalman Filter for sensor fusion (publishes `odom → base_link`)
2. `madgwick_filter_node` - IMU orientation filter
3. `robot_state_publisher` - Static URDF transforms

### Measured Rates

| Component | Expected | Actual | Status |
|-----------|----------|--------|--------|
| EKF frequency parameter | 50 Hz | 50 Hz | ✓ Configured correctly |
| IMU data (`/imu/corrected_data`) | 30+ Hz | ~43 Hz | ✓ Good |
| Wheel odometry (`/odom`) | 30+ Hz | ~22 Hz | ✓ Acceptable |
| Overall `/tf` topic | 30+ Hz | ~31 Hz | ✓ Good |
| `odom → base_link` transform | 30+ Hz | **1 Hz** | ✗ **CRITICAL ISSUE** |
| `base_link` frame (tf2_monitor) | 30+ Hz | 10 Hz | ⚠ Slow |

### The Mystery

**The core problem:** The `odom → base_link` transform is only updating at **1 Hz**, despite:
- EKF configured to run at 50 Hz ✓
- Input sensors publishing at good rates (22-43 Hz) ✓
- Overall `/tf` topic publishing at 31 Hz ✓
- EKF's `publish_tf` parameter set to `True` ✓

**This doesn't make sense!** The EKF should be publishing the transform at its configured frequency.

---

## Current Hypothesis

**Theory:** The EKF might not be the actual publisher of the `odom → base_link` transform, OR there's another node overriding it at 1 Hz.

**Evidence:**
- EKF has all the right configuration
- EKF is receiving sensor data at good rates
- But the transform it's supposed to publish is inexplicably slow
- The tf2_monitor shows `base_link` at only 10 Hz (not 50 Hz as EKF is configured)

**Possible causes:**
1. Frame name mismatch - EKF might be publishing to different frame names
2. Another node is overriding the EKF's transform
3. EKF is getting blocked/throttled somehow
4. There's a configuration issue we haven't found yet

---

## Data Still Needed

### Critical checks we were about to do:

```bash
# 1. Verify EKF frame names match what we expect
ros2 param get /ekf_filter_node odom_frame
ros2 param get /ekf_filter_node base_link_frame

# 2. Check if EKF is actually publishing what we think it is
ros2 topic echo /odom --field header.frame_id
ros2 topic echo /odom --field child_frame_id

# 3. Look for other nodes that might be publishing odom → base_link
ros2 node list
# Then check each node for TF publishers

# 4. Check for any warnings/errors from EKF itself
ros2 topic echo /diagnostics | grep ekf

# 5. Look at EKF's full parameter dump
ros2 param dump /ekf_filter_node
```

---

## Files Created

### `ros2_tf_diagnostics.py`
- **Location:** Should be in `/mnt/user-data/outputs/ros2_tf_diagnostics.py`
- **Purpose:** Automated diagnostic script that collects all timing data
- **Usage:** 
  ```bash
  ./ros2_tf_diagnostics.py
  ```
- **What it does:**
  - Identifies TF publishers
  - Measures topic rates
  - Runs tf2_monitor
  - Tests odom → base_link specifically
  - Checks EKF configuration
  - Provides recommendations

### Original error logs
- **Location:** `/mnt/user-data/uploads/1761572068387_pasted-content-1761572068385.txt`
- Contains the full error output showing repeated message drops

---

## Key Commands Reference

### Quick diagnostics:
```bash
# Check specific transform rate
ros2 run tf2_ros tf2_echo odom base_link

# Monitor all transforms
ros2 run tf2_ros tf2_monitor

# Check topic rates
ros2 topic hz /tf
ros2 topic hz /odom
ros2 topic hz /imu/corrected_data

# See TF publishers
ros2 topic info /tf --verbose

# Check EKF params
ros2 param list /ekf_filter_node
ros2 param get /ekf_filter_node frequency
ros2 param get /ekf_filter_node publish_tf
```

---

## Next Steps for Troubleshooting

### Immediate actions:
1. **Verify frame names** - Check if EKF's `odom_frame` and `base_link_frame` parameters match what the system expects
2. **Check /odom topic** - Verify what frames the EKF is actually publishing in its odometry message
3. **Look for conflicting publishers** - Another node might be publishing the transform at 1 Hz and overriding EKF
4. **Check EKF status** - Look for any warnings or errors from the EKF node itself

### Potential fixes (once root cause found):
1. **If frame name mismatch:** Update EKF config to use correct frame names
2. **If conflicting publisher:** Disable the slow publisher or fix its rate
3. **If EKF throttling:** Check CPU usage, reduce EKF update rate to match reality
4. **If sensor timing:** Adjust `transform_tolerance` in Nav2 params as workaround

---

## Important Context

- **Robot type:** Physical robot (not simulation)
- **User's workspace:** `~/ros2_ws`
- **Platform:** Ubuntu (vmubuntu hostname)
- **ROS2 distro:** Not explicitly stated, but using standard Nav2 stack

### Configuration files likely locations:
- EKF config: Look for `ekf.yaml` or similar in config directory
- Nav2 params: Look for nav2_params.yaml or similar
- Launch files: Check for robot bringup launch files

---

## Summary

We have a classic "the data says this should work but it doesn't" situation. The EKF is perfectly configured and receiving good sensor data, but somehow the transform it's supposed to publish at 50 Hz is only showing up at 1 Hz. The smoking gun will likely be found by:
1. Checking if the EKF's frame name parameters match reality
2. Finding out if there's another node secretly publishing that transform
3. Looking at the actual `/odom` topic to see what the EKF is really doing

The diagnostic script should help collect all this data systematically.