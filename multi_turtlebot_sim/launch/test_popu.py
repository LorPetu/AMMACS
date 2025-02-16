#!/usr/bin/env python3

import subprocess
import signal
import os
import shutil
import tkinter as tk
from tkinter import simpledialog

# Define the bag directory (this should match what's inside your launch file)
rosbag_root = "rosbags"

# Global variable to store the latest rosbag directory name
latest_rosbag_dir = None

def get_latest_rosbag():
    """Find the most recent rosbag directory based on creation time."""
    global latest_rosbag_dir
    try:
        # Get all directories inside rosbags
        dirs = [d for d in os.listdir(rosbag_root) if os.path.isdir(os.path.join(rosbag_root, d))]
        if not dirs:
            return None
        # Sort directories by creation time (latest first)
        latest_rosbag_dir = max(dirs, key=lambda d: os.path.getctime(os.path.join(rosbag_root, d)))
        return latest_rosbag_dir
    except Exception as e:
        print(f"Error finding latest rosbag: {e}")
        return None

def rename_rosbag():
    """Prompt user to rename the last rosbag directory."""
    global latest_rosbag_dir
    if latest_rosbag_dir is None:
        print("No rosbag found to rename.")
        return
    
    # Show a pop-up to get a new name
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    new_name = simpledialog.askstring("Rename Rosbag", f"Rename rosbag '{latest_rosbag_dir}' to:")
    
    if new_name:
        old_path = os.path.join(rosbag_root, latest_rosbag_dir)
        new_path = os.path.join(rosbag_root, new_name)
        shutil.move(old_path, new_path)
        print(f"Renamed rosbag to: {new_name}")

def signal_handler(sig, frame):
    """Handle Ctrl+C (SIGINT) by prompting for renaming."""
    print("\nCtrl+C detected! Checking for rosbag rename...")
    
    if get_latest_rosbag():
        rename_rosbag()
    else:
        print("No rosbag directory found.")

    print("Exiting...")
    exit(0)

# Attach signal handler
signal.signal(signal.SIGINT, signal_handler)

def main():
    """Launch the ROS 2 simulation and monitor for Ctrl+C."""
    try:
        process = subprocess.Popen(["ros2", "launch", "multi_turtlebot_sim", "multi_robot_test.launch.py"])
        process.wait()
    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")

if __name__ == "__main__":
    main()
