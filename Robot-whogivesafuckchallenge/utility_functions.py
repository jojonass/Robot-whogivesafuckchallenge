
import tkinter as tk
from eye_contact_detector import EyeContactDetector
import cv2
from PIL import Image, ImageTk



import time
import json
import os
import urx
import logging
from threading import Thread

# --- UR Configuration (REPLACE IP) ---
ROBOT_IP = "192.168.1.100"  # REPLACE with your actual robot IP address
DEFAULT_ACCEL = 0.5
DEFAULT_VEL = 0.5

# Set logging level for URX
logging.basicConfig(level=logging.WARN)

# --- GLOBAL CONFIGURATION AND DATA STRUCTURES ---
STATE_FILE = "robot_state.json"
ACTION_CHUNKS_FILE = "robot_action_chunks.json"

# Global variable to hold loaded action data
ACTION_CHUNKS_DATA = {}


# --- Mock Gripper Class (Minimal) ---
class Gripper:
    """Simulates a gripper device needed by the playback function."""

    def __init__(self):
        self.state = "open"

    def open_gripper(self):
        print("  [GRIPPER] Opening...")
        self.state = "open"

    def close_gripper(self):
        print("  [GRIPPER] Closing...")
        self.state = "closed"

    def get_state(self):
        return self.state


# --- Robot Connection ---

def connect_robot():
    """Connects to the UR robot and returns the robot and a mock gripper instance."""
    try:
        rob = urx.Robot(ROBOT_IP, use_rt=True)
        time.sleep(0.2)
        # Ensure TCP is set appropriately for your setup
        rob.set_tcp((0, 0, 0, 0, 0, 0))
        print(f"Successfully connected to robot at {ROBOT_IP}")

        # Instantiate the mock gripper
        gripper = MockGripper()

        return rob, gripper
    except Exception as e:
        print(f"ERROR: Could not connect to robot at {ROBOT_IP}. Details: {e}")
        return None, None


# --- JSON & State Management Functions ---

def load_action_chunks():
    """
    Loads action chunks from the external JSON file defined by ACTION_CHUNKS_FILE.
    """
    global ACTION_CHUNKS_DATA

    try:
        with open(ACTION_CHUNKS_FILE, 'r') as f:
            ACTION_CHUNKS_DATA = json.load(f)
        print(f"Action chunks loaded successfully from {ACTION_CHUNKS_FILE}. Total chunks: {len(ACTION_CHUNKS_DATA)}")
    except FileNotFoundError:
        print(f"ERROR: Action chunk file not found at '{ACTION_CHUNKS_FILE}'. Please ensure the file exists.")
    except json.JSONDecodeError:
        print(f"ERROR: Failed to decode JSON from '{ACTION_CHUNKS_FILE}'. Check file format.")
    except Exception as e:
        print(f"An unexpected error occurred while loading action chunks: {e}")


def get_chunk(key):
    """Retrieves an action chunk using standard dictionary lookup."""
    return ACTION_CHUNKS_DATA.get(key, f"ERROR: Chunk '{key}' not found in ACTION_CHUNKS_DATA.")


def initialize_state():
    """Initializes or resets the robot state file."""
    initial_state = {
        "current_slide_id": 0,
        "human_input_ready": False,
        "is_running": False
    }
    with open(STATE_FILE, 'w') as f:
        json.dump(initial_state, f)


def read_state():
    """Reads the current robot state."""
    try:
        with open(STATE_FILE, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        initialize_state()
        return read_state()
    except Exception as e:
        print(f"Warning: Corrupt state file read error: {e}. Reinitializing.")
        initialize_state()
        return read_state()


def update_robot_state(slide_id=None, key=None, value=None, clear_human_input=False):
    """Updates the robot state file."""
    state = read_state()
    if slide_id is not None:
        state["current_slide_id"] = slide_id
    if key and value is not None:
        state[key] = value

    # Only reset human_input_ready if specifically instructed by the robot thread
    if clear_human_input:
        state["human_input_ready"] = False

    with open(STATE_FILE, 'w') as f:
        json.dump(state, f)


import json
import os


def record_home_position(robot, gripper, file_path="home_position.json"):
    """
    Records the robot's current joint positions as the home position
    and saves it to a JSON file.

    Parameters:
    - robot: the robot object
    - gripper: the gripper object (optional, can save gripper state too)
    - file_path: path to save the home position JSON
    """
    if not robot:
        print("Robot instance not provided. Cannot record home position.")
        return False

    try:
        # Get current joint positions
        joint_positions = robot.get_actual_joint_positions()  # UR robot API
        gripper_position = None
        if gripper:
            # Optional: read gripper state if supported
            gripper_position = gripper.get_position() if hasattr(gripper, "get_position") else None

        # Structure data
        home_data = {
            "joints": joint_positions,
            "gripper": gripper_position
        }

        # Save to file
        with open(file_path, "w") as f:
            json.dump(home_data, f, indent=4)

        print(f"Home position recorded and saved to {file_path}.")
        return True

    except Exception as e:
        print(f"Failed to record home position: {e}")
        return False


import json
import os

HOME_FILE = "home_position.json"

def record_home_position(robot, gripper=None, file_path=HOME_FILE):
    """
    Records the robot's current joint positions as home.
    """
    try:
        joints = robot.get_actual_joint_positions()
        gripper_pos = gripper.get_position() if gripper and hasattr(gripper, "get_position") else None
        data = {"joints": joints, "gripper": gripper_pos}
        with open(file_path, "w") as f:
            json.dump(data, f, indent=4)
        print(f"[INFO] Home position recorded to {file_path}")
        return True
    except Exception as e:
        print(f"[ERROR] Failed to record home position: {e}")
        return False

def load_home_position(file_path=HOME_FILE):
    """
    Loads the saved home position.
    """
    if not os.path.exists(file_path):
        print(f"[WARNING] Home position file {file_path} not found")
        return None
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
        return data
    except Exception as e:
        print(f"[ERROR] Failed to load home position: {e}")
        return None

def is_at_home(robot, tolerance=0.01, file_path=HOME_FILE):
    """
    Checks if the robot is at the recorded home position.
    """
    home = load_home_position(file_path)
    if not home:
        print("[WARNING] Home position not recorded yet.")
        return False
    try:
        current = robot.get_actual_joint_positions()
        diffs = [abs(c - h) for c, h in zip(current, home["joints"])]
        return all(d < tolerance for d in diffs)
    except Exception as e:
        print(f"[ERROR] Failed to check home position: {e}")
        return False
