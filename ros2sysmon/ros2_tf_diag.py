#!/usr/bin/env python3
"""
ROS2 TF Timing Diagnostics Tool - Analyzes transform timing to diagnose navigation issues
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import glob
import os
import re
import subprocess
import sys
import time
from typing import Dict, List, Optional


class Colors:
    """Empty class for compatibility - no colors in output"""
    HEADER = ""
    OKBLUE = ""
    OKCYAN = ""
    OKGREEN = ""
    WARNING = ""
    FAIL = ""
    ENDC = ""
    BOLD = ""
    UNDERLINE = ""


def run_command(cmd: List[str], timeout: int = 5) -> Optional[str]:
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.stdout
    except subprocess.TimeoutExpired:
        return None
    except Exception as e:
        print(f"Error running command {' '.join(cmd)}: {e}")
        return None


def get_tf_publishers() -> List[Dict[str, str]]:
    """Get all nodes publishing to /tf"""
    print(f"\nChecking TF Publishers...")
    print(f"Why: /tf is the central topic for all coordinate transforms in ROS2.")
    print(f"     We need to identify which nodes are publishing transforms.")
    output = run_command(["ros2", "topic", "info", "/tf", "--verbose"], timeout=3)

    publishers = []
    if output:
        lines = output.split("\n")
        for line in lines:
            if "Node name:" in line:
                node_name = line.split("Node name:")[1].strip()
                publishers.append({"node": node_name})

    return publishers


def get_topic_hz(topic: str, duration: int = 3) -> Optional[float]:
    """Measure topic publishing rate"""
    print(f"  Measuring {topic} (waiting {duration}s)...", end="", flush=True)

    # Run ros2 topic hz for specified duration
    try:
        process = subprocess.Popen(
            ["ros2", "topic", "hz", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        time.sleep(duration)
        process.terminate()
        try:
            stdout, _ = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, _ = process.communicate()

        # Parse the average rate
        for line in stdout.split("\n"):
            if "average rate:" in line:
                rate = float(line.split("average rate:")[1].split()[0])
                print(f" {rate:.2f} Hz")
                return rate

        print(f" No data")
        return None

    except Exception as e:
        print(f" Error: {e}")
        return None


def get_tf_tree_from_view_frames() -> Dict[str, Dict]:
    """Get all transform rates from view_frames"""
    print(
        f"\nGetting TF Tree data (via view_frames)..."
    )
    print(f"Why: This shows update rates for all transforms in the TF tree.")
    print(f"     Helps identify which transforms are updating too slowly.")

    try:
        result = subprocess.run(
            ["ros2", "run", "tf2_tools", "view_frames"],
            capture_output=True,
            text=True,
            timeout=8
        )

        transforms = {}

        if "frame_yaml=" in result.stdout:
            yaml_str = result.stdout.split("frame_yaml=")[1].strip('")')
            lines = yaml_str.split("\\n")
            current_frame = None

            for line in lines:
                line = line.strip()
                # Frame name line
                if line and ":" not in line and re.match(r"^[a-zA-Z0-9_]+$", line):
                    current_frame = line
                    transforms[current_frame] = {}
                # Data lines
                elif current_frame and ":" in line:
                    parts = line.split(":", 1)
                    if len(parts) == 2:
                        key = parts[0].strip()
                        value = parts[1].strip().strip("'\"")
                        transforms[current_frame][key] = value

        if transforms:
            print(f"  Found {len(transforms)} transforms")
        else:
            print(f"  No transform data collected")

        return transforms

    except subprocess.TimeoutExpired:
        print(f"  view_frames timed out, skipping...")
        return {}
    except Exception as e:
        print(f"  Error: {e}")
        return {}


def test_specific_transform(
    parent: str, child: str, duration: int = 5
) -> Optional[float]:
    """Test a specific transform and measure its update rate"""
    print(f"\nTesting {parent} → {child} transform...")
    print(f"Why: The {parent}→{child} transform is critical for navigation.")
    print(f"     It needs to update at 20+ Hz for smooth robot control.")
    print(f"  Measuring for {duration}s...", end="", flush=True)

    process = None
    try:
        # Use timeout command to automatically kill tf2_echo after duration
        process = subprocess.Popen(
            ["timeout", str(duration + 1), "ros2", "run", "tf2_ros", "tf2_echo", parent, child],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        # Wait for timeout to kill it
        stdout, _ = process.communicate()

        # Parse timestamps from output
        timestamps = []
        for line in stdout.split("\n"):
            if "At time" in line:
                try:
                    timestamp = float(line.split("At time")[1].strip())
                    timestamps.append(timestamp)
                except (ValueError, IndexError):
                    pass

        if len(timestamps) >= 2:
            # Calculate average rate
            time_diffs = [
                timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)
            ]
            avg_period = sum(time_diffs) / len(time_diffs)
            rate = 1.0 / avg_period if avg_period > 0 else 0

            print(f" Done")
            status = "GOOD" if rate > 20 else "TOO SLOW"
            print(f"  Measured rate: {rate:.2f} Hz ({status})")
            return rate
        else:
            print(f" Failed")
            print(f"  *** TRANSFORM NOT AVAILABLE ***")
            print(f"  Check if '{parent}' and '{child}' are the correct frame names.")
            return None

    except Exception as e:
        print(f"  Error: {e}")
        if process:
            try:
                process.kill()
            except (OSError, AttributeError):
                pass
        return None


def identify_transform_publisher(parent: str, child: str) -> Optional[str]:
    """Identify which node is publishing a specific transform"""
    try:
        # Use view_frames to get the broadcaster info
        result = subprocess.run(
            ["ros2", "run", "tf2_tools", "view_frames"],
            capture_output=True,
            text=True,
            timeout=8
        )

        if "frame_yaml=" in result.stdout:
            yaml_str = result.stdout.split("frame_yaml=")[1].strip('")')
            lines = yaml_str.split("\\n")
            current_frame = None

            for line in lines:
                line = line.strip()
                if line and ":" not in line:
                    current_frame = line
                elif current_frame == child:
                    if "broadcaster:" in line:
                        broadcaster = line.split("broadcaster:")[1].strip().strip("'\"")
                        return broadcaster
                    elif "parent:" in line:
                        parent_frame = line.split("parent:")[1].strip().strip("'\"")
                        if parent_frame != parent:
                            return None

        return None

    except Exception:
        return None


def get_transform_rate_from_view_frames(child: str) -> Optional[float]:
    """Get transform rate using view_frames (alternative method)"""
    try:
        result = subprocess.run(
            ["ros2", "run", "tf2_tools", "view_frames"],
            capture_output=True,
            text=True,
            timeout=8
        )

        # Parse the frame_yaml from the output
        if "frame_yaml=" in result.stdout:
            yaml_str = result.stdout.split("frame_yaml=")[1].strip('")')

            # Look for the child frame and its rate
            lines = yaml_str.split("\\n")
            current_frame = None

            for line in lines:
                line = line.strip()
                if line and ":" not in line:
                    current_frame = line
                elif current_frame == child and "rate:" in line:
                    rate_str = line.split("rate:")[1].strip()
                    try:
                        rate = float(rate_str)
                        return rate
                    except ValueError:
                        pass

        return None

    except Exception:
        return None


def detect_base_frame() -> str:
    """Detect whether the system uses base_link or base_footprint as the primary base frame"""
    print(f"\nDetecting base frame...")

    # Check what the EKF is publishing
    output = run_command(["ros2", "topic", "echo", "/tf", "--once"], timeout=3)

    if output:
        if "base_footprint" in output:
            print(f"  Detected: base_footprint (with static transform to base_link)")
            return "base_footprint"
        elif "base_link" in output:
            print(f"  Detected: base_link")
            return "base_link"

    # Default to base_link if we can't detect
    print(f"  Using default: base_link")
    return "base_link"


def get_node_param(node: str, param: str) -> Optional[str]:
    """Get a parameter value from a node"""
    output = run_command(["ros2", "param", "get", node, param], timeout=2)
    if output:
        # Extract value from output like "String value is: something"
        for line in output.split("\n"):
            if "value is:" in line.lower():
                return line.split(":", 1)[1].strip()
    return None


def check_ekf_config() -> Dict[str, str]:
    """Check EKF filter node configuration"""
    print(f"\nChecking EKF Configuration...")
    print(f"Why: The EKF (Extended Kalman Filter) fuses sensor data to estimate robot pose.")
    print(f"     Its configuration directly affects the odom→base_link transform rate.")

    config = {}
    params_to_check = [
        "frequency",
        "publish_tf",
        "odom_frame",
        "base_link_frame",
        "imu0",
        "odom0",
    ]

    for param in params_to_check:
        value = get_node_param("/ekf_filter_node", param)
        if value:
            print(f"  {param}: {value}")
            config[param] = value
        else:
            print(f"  {param}: Not found")

    return config


def print_summary_table(data: Dict):
    """Print a nice summary table of all collected data"""
    print(f"\n{'=' * 80}")
    print(f"DIAGNOSTIC SUMMARY")
    print(f"{'=' * 80}\n")

    # TF Publishers
    if "tf_publishers" in data:
        print(f"TF PUBLISHERS:")
        print(f"  (These nodes broadcast coordinate frame transforms)")
        for pub in data["tf_publishers"]:
            print(f"  • {pub['node']}")

    # Topic Rates
    if "topic_rates" in data:
        print(f"\nTOPIC PUBLISHING RATES:")
        print(f"  (Target: 30+ Hz for good performance, 20+ Hz minimum)")
        for topic, rate in data["topic_rates"].items():
            if rate is None:
                status = f"No data"
            elif rate > 20:
                status = f"{rate:.1f} Hz ✓"
            elif rate > 10:
                status = f"{rate:.1f} Hz ⚠"
            else:
                status = f"{rate:.1f} Hz ✗"
            print(f"  {topic:30s} {status}")

    # Transform Rates
    if "tf_monitor" in data and data["tf_monitor"]:
        print(f"\nTRANSFORM UPDATE RATES (FROM VIEW_FRAMES):")
        print(f"  (How often each coordinate frame gets updated)")
        for frame, info in data["tf_monitor"].items():
            rate_str = info.get("rate", info.get("Average rate", "Unknown"))
            try:
                rate = float(rate_str)
                if rate > 1000:  # Static transform
                    status = f"Static"
                elif rate > 20:
                    status = f"{rate:.1f} Hz ✓"
                elif rate > 5:
                    status = f"{rate:.1f} Hz ⚠"
                else:
                    status = f"{rate:.1f} Hz ✗"
            except (ValueError, TypeError):
                status = rate_str

            print(f"  {frame:25s} {status}")

    # Specific Transform Test
    if "odom_base_link_rate" in data:
        base_frame = data.get("base_frame", "base_link")
        print(f"\nCRITICAL TRANSFORM (ODOM → {base_frame.upper()}):")
        print(f"  (This transform is essential for navigation - it tells Nav2 where the robot is)")

        if base_frame == "base_footprint":
            print(f"  Note: Your system uses base_footprint with a static transform to base_link")

        # Show which node is publishing this transform
        publisher = data.get("base_frame_publisher")
        if publisher and publisher != "default_authority":
            print(f"  Published by: {publisher} (inferred from /tf publishers)")
        elif publisher == "default_authority":
            print(f"  Published by: default_authority (multiple publishers)")
        else:
            # Show what nodes ARE publishing to /tf to help user
            tf_pubs = data.get("tf_publishers", [])
            if tf_pubs:
                pub_names = ", ".join([p["node"] for p in tf_pubs[:3]])
                print(f"  Publisher: Unknown (nodes publishing to /tf: {pub_names})")
            else:
                print(f"  Publisher: Unknown (no nodes publishing to /tf)")

        rate = data["odom_base_link_rate"]  # From view_frames (PRIMARY)
        rate_tf2echo = data.get("odom_base_link_rate_tf2echo")  # From tf2_echo (may be wrong for stationary robots)

        # Display PRIMARY measurement (from view_frames)
        if rate is None:
            status = "*** TRANSFORM NOT PUBLISHED - NAVIGATION IMPOSSIBLE ***"
            print(f"  Publishing Rate:    {status}")

            # Identify which node should be publishing this transform
            print(f"\n  TROUBLESHOOTING:")
            publisher = data.get("base_frame_publisher")

            if publisher and publisher != "default_authority":
                print(f"  Expected publisher: {publisher}")
                print(f"  → Check if the '{publisher}' node is running")
                print(f"  → Check if it's configured to publish this transform")
            else:
                print(f"  Expected publisher: Usually 'ekf_filter_node' or 'robot_localization'")
                print(f"  → Check if your EKF/localization node is running")

            # Show what's currently publishing to /tf
            publishers = data.get("tf_publishers", [])
            if publishers:
                print(f"\n  Nodes currently publishing to /tf:")
                for pub in publishers:
                    print(f"    - {pub['node']}")
            else:
                print(f"\n  No nodes are publishing to /tf at all!")
        elif rate > 20:
            status = f"{rate:.1f} Hz ✓ GOOD"
            print(f"  Publishing Rate:    {status}")
        elif rate > 10:
            status = f"{rate:.1f} Hz ⚠ MARGINAL"
            print(f"  Publishing Rate:    {status}")
        else:
            status = f"{rate:.1f} Hz ✗ TOO SLOW"
            print(f"  Publishing Rate:    {status}")

            # Show which node is publishing (for slow transforms)
            print(f"\n  TROUBLESHOOTING:")
            publisher = data.get("base_frame_publisher")
            if publisher and publisher != "default_authority":
                print(f"  Currently published by: {publisher}")
                print(f"  → Check the '{publisher}' node configuration")
                print(f"  → Check input sensor rates (IMU, odometry)")
            else:
                print(f"  → Check your EKF node configuration and input sensor rates")

        # Display OPTIONAL tf2_echo measurement (for debugging only)
        if rate_tf2echo is not None:
            if rate_tf2echo > 20:
                status_tf2 = f"{rate_tf2echo:.1f} Hz"
            elif rate_tf2echo > 1:
                status_tf2 = f"{rate_tf2echo:.1f} Hz (may be low if robot stationary)"
            else:
                status_tf2 = f"{rate_tf2echo:.1f} Hz (robot likely stationary)"
            print(f"  tf2_echo check:     {status_tf2}")

        # Show chain result if tested
        if "odom_base_link_via_chain" in data and data["odom_base_link_via_chain"]:
            chain_rate = data["odom_base_link_via_chain"]  # From view_frames (PRIMARY)
            chain_rate_tf2echo = data.get("odom_base_link_via_chain_tf2echo")

            print(f"\nFULL CHAIN (ODOM → BASE_FOOTPRINT → BASE_LINK):")

            # PRIMARY measurement from view_frames
            if chain_rate > 20:
                status = f"{chain_rate:.1f} Hz ✓ GOOD"
            elif chain_rate > 10:
                status = f"{chain_rate:.1f} Hz ⚠ MARGINAL"
            else:
                status = f"{chain_rate:.1f} Hz ✗ TOO SLOW"
            print(f"  Publishing Rate:    {status}")

            # OPTIONAL tf2_echo measurement
            if chain_rate_tf2echo:
                if chain_rate_tf2echo > 20:
                    status_tf2 = f"{chain_rate_tf2echo:.1f} Hz"
                elif chain_rate_tf2echo > 1:
                    status_tf2 = f"{chain_rate_tf2echo:.1f} Hz (may be low if robot stationary)"
                else:
                    status_tf2 = f"{chain_rate_tf2echo:.1f} Hz (robot likely stationary)"
                print(f"  tf2_echo check:     {status_tf2}")

    # EKF Config
    if "ekf_config" in data:
        print(f"\nEKF CONFIGURATION:")
        for param, value in data["ekf_config"].items():
            print(f"  {param:20s} {value}")


def print_recommendations(data: Dict):
    """Analyze data and print recommendations"""
    print(f"\n{'=' * 80}")
    print(f"RECOMMENDATIONS")
    print(f"{'=' * 80}\n")
    print(f"Analyzing collected data to identify issues...\n")

    issues = []

    # Check odom → base_link rate
    odom_rate = data.get("odom_base_link_rate")
    if odom_rate is None:
        issues.append(
            {
                "severity": "CRITICAL",
                "issue": "odom → base_link transform is NOT being published!",
                "fix": "Start the EKF filter node or the node responsible for publishing this transform. "
                      "Check: 1) Is ekf_filter_node running? 2) Is publish_tf=true? 3) Are frame names correct?",
                "explanation": "Without this transform, Nav2 has no idea where the robot is. Navigation is completely broken. "
                              "This is not a performance issue - the transform simply doesn't exist.",
            }
        )
    elif odom_rate < 10:
            issues.append(
                {
                    "severity": "CRITICAL",
                    "issue": f"odom → base_link transform only updating at {odom_rate:.1f} Hz",
                    "fix": "This is causing your navigation errors. The transform needs to be at least 20 Hz.",
                    "explanation": "Nav2 requires frequent pose updates to plan paths and control the robot. "
                                  "Below 10 Hz, the robot appears to 'jump' between positions, causing planning failures.",
                }
            )

    # Check topic rates
    if "topic_rates" in data:
        for topic, rate in data["topic_rates"].items():
            if rate and rate < 20:
                issues.append(
                    {
                        "severity": "WARNING",
                        "issue": f"{topic} publishing at only {rate:.1f} Hz",
                        "fix": "Consider increasing the rate of this topic to 30+ Hz",
                        "explanation": f"Sensor data from {topic} feeds into the EKF filter. "
                                      "Low input rates limit the EKF output rate, affecting transform quality.",
                    }
                )

    # Check EKF config vs actual performance
    ekf_freq = data.get("ekf_config", {}).get("frequency")
    odom_rate = data.get("odom_base_link_rate")

    if ekf_freq and odom_rate:
        try:
            ekf_hz = float(ekf_freq.replace(" Hz", ""))
            if ekf_hz > odom_rate * 2:
                issues.append(
                    {
                        "severity": "WARNING",
                        "issue": f"EKF configured for {ekf_hz} Hz but only outputting {odom_rate:.1f} Hz",
                        "fix": "EKF output is limited by input sensor rates. Check wheel odometry and IMU rates.",
                        "explanation": "The EKF can't produce output faster than its slowest input. "
                                      "If IMU or odometry data arrives slowly, EKF output will be slow too.",
                    }
                )
        except (ValueError, AttributeError):
            pass

    if not issues:
        print(f"✓ No critical issues detected!")
        print(f"Your TF tree appears healthy. All transforms are updating at good rates.\n")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"{issue['severity']} Issue {i}:")
            print(f"  Problem: {issue['issue']}")
            if "explanation" in issue:
                print(f"  Why this matters: {issue['explanation']}")
            print(f"  How to fix: {issue['fix']}\n")


def main():
    """Main diagnostic routine"""
    print(f"\n")
    print("╔════════════════════════════════════════════════════════════════════╗")
    print("║         ROS2 TF Timing Diagnostics Tool                           ║")
    print("╚════════════════════════════════════════════════════════════════════╝")
    print(f"\n")

    print(f"This tool diagnoses transform (TF) timing issues that cause navigation problems.")
    print(f"It will:")
    print(f"  1. Identify which nodes publish transforms")
    print(f"  2. Measure how fast key topics are updating")
    print(f"  3. Check transform update rates in the TF tree")
    print(f"  4. Auto-detect your base frame (base_link or base_footprint)")
    print(f"  5. Test the critical odom→base transform")
    print(f"  6. Verify EKF configuration")
    print(f"Total time: ~20-25 seconds\n")

    data = {}

    # 1. Get TF publishers
    data["tf_publishers"] = get_tf_publishers()

    # 2. Measure topic rates
    print(f"\nMeasuring Topic Rates...")
    print(f"Why: Topic rates affect transform update frequency.")
    print(f"     Sensor data must arrive fast enough for smooth robot control (30+ Hz ideal).")

    # Get available topics first
    print("  Detecting available topics...")
    topic_list_output = run_command(["ros2", "topic", "list"], timeout=2)
    available_topics = set(topic_list_output.split("\n")) if topic_list_output else set()

    # Check for these topics in priority order
    topics_to_check = []

    # Always check /tf
    topics_to_check.append("/tf")

    # Check for odometry (prefer filtered over unfiltered)
    if "/odom" in available_topics:
        topics_to_check.append("/odom")
    elif "/odom/unfiltered" in available_topics:
        topics_to_check.append("/odom/unfiltered")

    # Check for IMU (prefer corrected, then filtered, then raw)
    if "/imu/corrected_data" in available_topics:
        topics_to_check.append("/imu/corrected_data")
    elif "/imu/data" in available_topics:
        topics_to_check.append("/imu/data")
    elif "/imu/data_raw" in available_topics:
        topics_to_check.append("/imu/data_raw")

    # Check for scan data
    if "/scan" in available_topics:
        topics_to_check.append("/scan")

    print(f"  Found {len(topics_to_check)} key topics to measure")

    data["topic_rates"] = {}
    for topic in topics_to_check:
        rate = get_topic_hz(topic, duration=3)
        data["topic_rates"][topic] = rate

    # 3. Get TF tree data from view_frames (COLLECTS ALL FRAME DATA AT ONCE)
    print(f"\nCollecting TF tree data...")
    print(f"This gathers all transform rates and publishers in one pass.")
    tf_tree_data = get_tf_tree_from_view_frames()
    data["tf_monitor"] = tf_tree_data

    # 4. Detect which base frame is being used
    base_frame = detect_base_frame()
    data["base_frame"] = base_frame

    # 5. Get transform rate from already-collected view_frames data (PRIMARY SOURCE)
    print(f"\nGetting transform rate for: odom → {base_frame}")
    if base_frame in tf_tree_data:
        rate_str = tf_tree_data[base_frame].get("rate", "")
        try:
            data["odom_base_link_rate"] = float(rate_str)
            print(f"  Rate from view_frames: {data['odom_base_link_rate']:.1f} Hz")
        except (ValueError, TypeError):
            data["odom_base_link_rate"] = None
            print("  Rate from view_frames: Unable to parse")
    else:
        data["odom_base_link_rate"] = None
        print("  Transform not found in TF tree")

    # 5b. OPTIONAL: Test with tf2_echo (may show lower rate if robot is stationary)
    # NOTE: tf2_echo only reports when transform VALUES change, not when published
    # So for stationary robots, this will show ~1 Hz even if publishing at 20+ Hz
    # We keep this for debugging but don't use it as primary measurement
    data["odom_base_link_rate_tf2echo"] = test_specific_transform("odom", base_frame, 5)

    # Also check odom -> base_link if we're using base_footprint
    if base_frame == "base_footprint":
        print("\nChecking the full chain: odom → base_link")

        # Extract base_link rate from already-collected view_frames data (PRIMARY)
        if "base_link" in tf_tree_data:
            rate_str = tf_tree_data["base_link"].get("rate", "")
            try:
                data["odom_base_link_via_chain"] = float(rate_str)
                print(f"  Rate from view_frames: {data['odom_base_link_via_chain']:.1f} Hz")
            except (ValueError, TypeError):
                data["odom_base_link_via_chain"] = None
                print("  Rate from view_frames: Unable to parse")
        else:
            data["odom_base_link_via_chain"] = None
            print("  Transform not found in TF tree")

        # OPTIONAL: Test with tf2_echo (for debugging)
        data["odom_base_link_via_chain_tf2echo"] = test_specific_transform("odom", "base_link", 5)

    # Store broadcaster info for troubleshooting
    # Note: view_frames often shows "default_authority" which isn't helpful
    # So we'll try to infer the publisher from the TF publishers list
    broadcaster = None
    if base_frame in tf_tree_data:
        broadcaster = tf_tree_data[base_frame].get("broadcaster", None)

    # If broadcaster is default_authority, try to identify actual publisher
    if broadcaster == "default_authority" or not broadcaster:
        # For odom->base_footprint, it's usually the EKF node
        tf_publishers = data.get("tf_publishers", [])
        likely_publishers = []
        for pub in tf_publishers:
            node_name = pub.get("node", "")
            # Look for localization/EKF nodes
            if any(keyword in node_name.lower() for keyword in ["ekf", "localization", "amcl"]):
                likely_publishers.append(node_name)

        if likely_publishers:
            data["base_frame_publisher"] = likely_publishers[0]
        else:
            data["base_frame_publisher"] = None
    else:
        data["base_frame_publisher"] = broadcaster

    # 6. Check EKF configuration
    data["ekf_config"] = check_ekf_config()

    # Print summary
    print_summary_table(data)

    # Print recommendations
    print_recommendations(data)

    print(f"\nDiagnostics complete!\n")

    print(f"UNDERSTANDING THE RESULTS:")
    print(f"• Green (✓): Everything is working well")
    print(f"• Yellow (⚠): Performance could be improved")
    print(f"• Red (✗): Critical issue - must be fixed for navigation to work")

    # Add extra warning if transform is missing
    if data.get("odom_base_link_rate") is None:
        print(f"\n╔══════════════════════════════════════════════════════════════════╗")
        print(f"║  CRITICAL: Navigation cannot work without odom→base_link!       ║")
        print(f"║  Fix this before attempting to navigate.                        ║")
        print(f"╚══════════════════════════════════════════════════════════════════╝")

    print(f"\nFor more help, check the ROS2 Navigation documentation at:")
    print(f"https://navigation.ros.org\n")


def cleanup_pdf_files():
    """Delete PDF files created by view_frames"""
    # view_frames creates files named frames_YYYY-MM-DD_HH.MM.SS.pdf
    pdf_files = glob.glob("frames_*.pdf")
    for pdf_file in pdf_files:
        try:
            os.remove(pdf_file)
        except OSError:
            pass  # Ignore errors if file doesn't exist or can't be deleted


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        sys.exit(1)
    finally:
        cleanup_pdf_files()
