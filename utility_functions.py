

import tkinter as tk
from eye_contact_detector import EyeContactDetector
import cv2
from PIL import Image, ImageTk


class EyeContactApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Eye Contact Detection")
        self.detector = EyeContactDetector()

        self.label = tk.Label(root, text="Eye Contact", font=("Helvetica", 24))
        self.label.pack()

        self.canvas = tk.Canvas(root, width=640, height=480)
        self.canvas.pack()

        self.quit_button = tk.Button(root, text="Quit", command=self.quit_program)
        self.quit_button.pack(side="left")

        self.calibrate_button = tk.Button(root, text="Calibrate", command=self.calibrate)
        self.calibrate_button.pack(side="right")

        self.update_frame()

    def update_frame(self):
        image, eye_contact, zone, proximity = self.detector.detect_eye_contact()

        if eye_contact:
            self.label.config(text="Eye Contact", fg="green")
        else:
            self.label.config(text="No Eye Contact", fg="red")

        # Convert the image to PhotoImage format for Tkinter
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img)

        self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
        self.canvas.img_tk = img_tk  # Keep a reference

        self.root.after(10, self.update_frame)  # Update frame every 10 ms

        return zone, proximity

    def calibrate(self):
        self.detector.calibrate()

    def quit_program(self):
        self.detector.release()
        self.root.quit()


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
class MockGripper:
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

