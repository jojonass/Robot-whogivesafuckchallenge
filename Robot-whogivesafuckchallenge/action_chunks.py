import urx
import time
import json
import logging
from math import pi
import os  # For checking file existence

# --- Configuration ---
ROBOT_IP = "192.168.1.100"  # REPLACE with your actual robot IP address
CALIBRATION_FILE = "recorded_action_chunks.json"
DEFAULT_ACCEL = 0.5
DEFAULT_VEL = 0.5

# Set logging level
logging.basicConfig(level=logging.WARN)


# --- Mock Gripper Class ---
class MockGripper:
    """Simulates a gripper device for recording and playback."""

    def __init__(self, rob):
        self.rob = rob
        self.state = "open"
        print("Mock Gripper initialized.")

    def open_gripper(self):
        print("  [GRIPPER] Opening...")
        time.sleep(0.5)  # Simulate time taken
        self.state = "open"

    def close_gripper(self):
        print("  [GRIPPER] Closing...")
        time.sleep(0.5)  # Simulate time taken
        self.state = "closed"

    def get_state(self):
        return self.state


# --- File Management Functions ---

def save_chunks(chunks, filename=CALIBRATION_FILE):
    """Saves the recorded action chunks to a JSON file."""
    try:
        with open(filename, 'w') as f:
            json.dump(chunks, f, indent=4)
        total_actions = sum(len(chunk["actions"]) for chunk in chunks)
        print(
            f"\nSuccessfully saved {len(chunks)} action chunk(s) containing {total_actions} total steps to {filename}")
    except Exception as e:
        print(f"Error saving chunks: {e}")


def load_chunks(filename=CALIBRATION_FILE):
    """Loads action chunks from a JSON file."""
    if not os.path.exists(filename):
        print(f"File '{filename}' not found. Starting with empty chunks list.")
        return []
    try:
        with open(filename, 'r') as f:
            chunks = json.load(f)
        total_actions = sum(len(chunk["actions"]) for chunk in chunks)
        print(f"Successfully loaded {len(chunks)} action chunk(s) from {filename} ({total_actions} total steps).")
        return chunks
    except Exception as e:
        print(f"Error loading chunks: {e}")
        return []


# --- Robot Control Functions ---

def connect_robot():
    """Connects to the UR robot and initializes the gripper mock."""
    try:
        rob = urx.Robot(ROBOT_IP, use_rt=True)
        time.sleep(0.2)
        rob.set_tcp((0, 0, 0, 0, 0, 0))  # Set TCP to 0,0,0
        print(f"Successfully connected to robot at {ROBOT_IP}")

        # Instantiate the mock gripper
        gripper = MockGripper(rob)

        return rob, gripper
    except Exception as e:
        print(f"Error connecting to robot: {e}")
        return None, None


def execute_action(rob, gripper, action):
    """Executes a single step (action dictionary)."""
    action_type = action.get("type")
    params = action.get("params", {})

    try:
        if action_type == "move_l":
            pose = params["pose"]
            a = params.get("a", DEFAULT_ACCEL)
            v = params.get("v", DEFAULT_VEL)
            print(f"  > Executing MOVE_L: {pose[:3]}...")
            rob.movel(pose, acc=a, vel=v)

        elif action_type == "move_j":
            pose = params["pose"]
            a = params.get("a", DEFAULT_ACCEL)
            v = params.get("v", DEFAULT_VEL)
            print(f"  > Executing MOVE_J (Joints): {pose}...")
            # For movej, pose must be joint angles in radians
            rob.movej(pose, acc=a, vel=v)

        elif action_type == "gripper_open":
            gripper.open_gripper()

        elif action_type == "gripper_close":
            gripper.close_gripper()

        elif action_type == "wait":
            duration = params.get("duration", 1.0)
            print(f"  > Executing WAIT for {duration} seconds.")
            time.sleep(duration)

        else:
            print(f"  > WARNING: Unknown action type: {action_type}")

    except urx.RobotException as e:
        print(f"  > ROBOT ERROR during action execution: {e}. Aborting chunk.")
        raise
    except Exception as e:
        print(f"  > GENERAL ERROR during action execution: {e}. Aborting chunk.")
        raise


def playback_chunk(rob, gripper, chunk):
    """Plays back a full action chunk (list of steps)."""
    if not rob or not chunk or not gripper:
        print("Cannot playback: Robot not connected, gripper not available, or chunk is empty.")
        return

    print(f"\n--- Starting Playback of Chunk: {chunk['name']} ({len(chunk['actions'])} steps) ---")

    for i, action in enumerate(chunk["actions"]):
        print(f"Step {i + 1}/{len(chunk['actions'])}: {action['type']}")
        try:
            execute_action(rob, gripper, action)
        except Exception:
            # An error occurred during execution (caught inside execute_action)
            return

    print(f"--- Chunk '{chunk['name']}' Completed Successfully. ---")


# --- Recording Interface ---

def record_new_chunk(rob, gripper, existing_chunks, chunk_name=None):
    """Interactive interface to record a new action chunk.

    If chunk_name is given, it is used directly (no prompt).
    Otherwise the user is asked to enter a name.
    """
    if not rob:
        print("Please connect to the robot first.")
        return existing_chunks

    # If no name is provided, ask the user
    if chunk_name is None:
        chunk_name = input("\nEnter a descriptive name for this new Action Chunk (e.g., 'Pick_Part_A'): ").strip()
        if not chunk_name:
            print("Chunk name cannot be empty. Aborting recording.")
            return existing_chunks

    new_chunk = {
        "name": chunk_name,
        "actions": []
    }

    print(f"\n--- Recording Action Chunk: '{chunk_name}' ---")
    print(
        "Commands: 'r' (MoveL), 'j' (MoveJ), 'go' (Gripper Open), 'gc' (Gripper Close), "
        "'w' (Wait), 'q' (Finish and Save), 'd' (Delete last step)"
    )

    while True:
        try:
            command = input(f"Action #{len(new_chunk['actions']) + 1} ({chunk_name}) > ").strip().lower()

            if command == 'r':  # Record MoveL (Cartesian Pose)
                pose = rob.getl()
                new_chunk["actions"].append({
                    "type": "move_l",
                    "params": {"pose": pose, "a": DEFAULT_ACCEL, "v": DEFAULT_VEL}
                })
                print(f"  > MOVE_L recorded (TCP Pose: {pose[:3]}).")

            elif command == 'j':  # Record MoveJ (Joint Angles)
                joints = rob.getj()
                new_chunk["actions"].append({
                    "type": "move_j",
                    "params": {"pose": joints, "a": DEFAULT_ACCEL, "v": DEFAULT_VEL}
                })
                print(f"  > MOVE_J recorded (Joints: {joints}).")

            elif command == 'go':  # Gripper Open
                new_chunk["actions"].append({"type": "gripper_open", "params": {}})
                print("  > GRIPPER_OPEN recorded.")

            elif command == 'gc':  # Gripper Close
                new_chunk["actions"].append({"type": "gripper_close", "params": {}})
                print("  > GRIPPER_CLOSE recorded.")

            elif command == 'w':  # Wait
                duration = input("  > Enter wait duration (seconds, default 1.0): ")
                duration = float(duration) if duration else 1.0
                new_chunk["actions"].append({"type": "wait", "params": {"duration": duration}})
                print(f"  > WAIT for {duration} seconds recorded.")

            elif command == 'q':
                break

            elif command == 'd':
                if new_chunk["actions"]:
                    removed_action = new_chunk["actions"].pop()
                    print(f"  > Last action ({removed_action['type']}) deleted.")
                else:
                    print("  > No actions to delete in current chunk.")

            else:
                print("  > Invalid command. Use 'r', 'j', 'go', 'gc', 'w', 'q', or 'd'.")

        except ValueError:
            print("  > Invalid input. Ensure numbers are entered correctly.")
        except Exception as e:
            print(f"  > An error occurred during recording: {e}")

    if new_chunk["actions"]:
        existing_chunks.append(new_chunk)
        save_chunks(existing_chunks)
    else:
        print("Chunk saved but is empty. Not adding to list.")

    return existing_chunks


def record_tool_chunks(rob, gripper, existing_chunks):
    """
    Guided recording of the tool position chunks needed by Robot_main.py
    for the A→B→C→D→C→A logic.

    Required chunks:

      A (tool-specific):
        - tool_location_m4
        - tool_location_m5
        - tool_location_m3

      B (common):
        - go_tool_square

      C (common):
        - go_tool_view
    """
    if not rob:
        print("Please connect to the robot first.")
        return existing_chunks

    required_chunks = [
        "tool_location_m4",
        "tool_location_m5",
        "tool_location_m3",
        "go_tool_square",
        "go_tool_view",
    ]

    print("\n=== Guided recording for tool-position chunks (A/B/C) ===")
    print("This will help you record the positions used in the main robot sequence:")
    print("  - A: tool_location_m4 / m5 / m3 (per tool)")
    print("  - B: go_tool_square (common)")
    print("  - C: go_tool_view   (common)")
    print("For each chunk, you can choose to record or skip.")
    print("Existing chunks will not be overwritten automatically; if you reuse a name,")
    print("you will just add another chunk entry in the JSON file.\n")

    for chunk_name in required_chunks:
        ans = input(f"Record chunk '{chunk_name}' now? [y/N]: ").strip().lower()
        if ans == 'y':
            print(f"\nRecording '{chunk_name}'. Position the UR arm as needed and add actions.")
            existing_chunks = record_new_chunk(rob, gripper, existing_chunks, chunk_name=chunk_name)
        else:
            print(f"Skipping '{chunk_name}'.")

    print("\nFinished guided recording of tool-position chunks.")
    return existing_chunks


# --- Main Application Loop ---

def main_menu():
    rob = None
    gripper = None
    # Load chunks on startup
    action_chunks = load_chunks()

    while True:
        print("\n==============================")
        print("=== UR Action Chunk Manager ===")
        print(f"Robot Status: {'Connected' if rob else 'Disconnected'}")
        print(f"Chunks Loaded: {len(action_chunks)}")
        print("==============================")
        print("1. Connect to Robot")
        print("2. Record New Action Chunk (Task)")
        print("3. Execute Action Chunk")
        print("4. Quit")
        print("5. Record M3/M4/M5 Tool-Position Chunks (A/B/C)")

        choice = input("Enter your choice (1-5): ").strip()

        if choice == '1':
            if rob:
                rob.close()
            rob, gripper = connect_robot()

        elif choice == '2':
            action_chunks = record_new_chunk(rob, gripper, action_chunks)

        elif choice == '3':
            if not rob or not action_chunks:
                print("Please connect to the robot AND load/record chunks first.")
                continue

            print("\nAvailable Chunks:")
            for i, chunk in enumerate(action_chunks):
                print(f"  {i + 1}: {chunk['name']} ({len(chunk['actions'])} steps)")

            try:
                chunk_index = int(input("Enter chunk number to execute: ")) - 1
                if 0 <= chunk_index < len(action_chunks):
                    chunk_to_execute = action_chunks[chunk_index]
                    playback_chunk(rob, gripper, chunk_to_execute)
                else:
                    print("Invalid chunk number.")
            except ValueError:
                print("Invalid input. Please enter a number.")

        elif choice == '4':
            print("Closing connection and exiting.")
            if rob:
                rob.close()
            break

        elif choice == '5':
            action_chunks = record_tool_chunks(rob, gripper, action_chunks)

        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main_menu()
