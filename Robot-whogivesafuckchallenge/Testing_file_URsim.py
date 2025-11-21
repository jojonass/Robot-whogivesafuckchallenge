import urx
import json
import os
import subprocess
import time
# --- Configuration ---
ROBOT_IP = "127.0.0.1"  # URSim default IP
CHUNKS_FILE = "test_action_chunks.json"
DEFAULT_ACC = 0.5
DEFAULT_VEL = 0.5


# --- CONFIGURATION ---
URSIM_FOLDER = r"C:\Users\97253\Downloads\URSim_VIRTUAL-5.12.6.1102099"  # Update to your URSim folder
URSIM_EXECUTABLE = "start.bat"  # The URSim start script (usually a batch file)
WAIT_TIME = 10  # Seconds to wait for URSim to boot up

def start_ursim():
    """Launch URSim via its batch file."""
    start_path = os.path.join(URSIM_FOLDER, URSIM_EXECUTABLE)

    if not os.path.exists(start_path):
        print(f"Error: '{start_path}' not found. Check your URSim folder.")
        return False

    print("Starting URSim...")
    subprocess.Popen([start_path], cwd=URSIM_FOLDER, shell=True)
    print(f"Waiting {WAIT_TIME} seconds for URSim to initialize...")
    time.sleep(WAIT_TIME)
    print("URSim should be running now.")
    return True

# --- Mock Gripper ---
class MockGripper:
    def __init__(self, rob):
        self.rob = rob
        self.state = "open"
        print("Mock gripper initialized.")

    def open_gripper(self):
        print("Gripper opening...")
        self.state = "open"
        time.sleep(0.3)

    def close_gripper(self):
        print("Gripper closing...")
        self.state = "closed"
        time.sleep(0.3)

    def get_state(self):
        return self.state

# --- File functions ---
def save_chunks(chunks, filename=CHUNKS_FILE):
    with open(filename, 'w') as f:
        json.dump(chunks, f, indent=4)
    print(f"Saved {len(chunks)} chunk(s) to {filename}.")

def load_chunks(filename=CHUNKS_FILE):
    if not os.path.exists(filename):
        return []
    with open(filename, 'r') as f:
        return json.load(f)

# --- Robot functions ---
def connect_robot():
    try:
        rob = urx.Robot(ROBOT_IP, use_rt=True)
        print(f"Connected to URSim at {ROBOT_IP}")
        gripper = MockGripper(rob)
        return rob, gripper
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None, None

def execute_action(rob, gripper, action):
    typ = action["type"]
    params = action.get("params", {})

    if typ == "move_l":
        rob.movel(params["pose"], acc=params.get("a", DEFAULT_ACC), vel=params.get("v", DEFAULT_VEL))
        print(f"Moved to {params['pose'][:3]}")
    elif typ == "move_j":
        rob.movej(params["pose"], acc=params.get("a", DEFAULT_ACC), vel=params.get("v", DEFAULT_VEL))
        print(f"Joints: {params['pose']}")
    elif typ == "gripper_open":
        gripper.open_gripper()
    elif typ == "gripper_close":
        gripper.close_gripper()
    elif typ == "wait":
        t = params.get("duration", 1)
        print(f"Waiting {t} s")
        time.sleep(t)

def playback_chunk(rob, gripper, chunk):
    print(f"\n--- Playing back chunk: {chunk['name']} ---")
    for a in chunk["actions"]:
        execute_action(rob, gripper, a)
    print(f"--- Chunk {chunk['name']} done ---\n")

# --- Recording a test chunk ---
def record_test_chunk(rob, gripper):
    chunk = {"name": "Test_Move", "actions": []}

    print("Move the robot to first position and press Enter...")
    input()
    chunk["actions"].append({"type": "move_l", "params": {"pose": rob.getl(), "a": DEFAULT_ACC, "v": DEFAULT_VEL}})

    print("Move the robot to second position and press Enter...")
    input()
    chunk["actions"].append({"type": "move_l", "params": {"pose": rob.getl(), "a": DEFAULT_ACC, "v": DEFAULT_VEL}})

    chunk["actions"].append({"type": "gripper_close", "params": {}})
    chunk["actions"].append({"type": "wait", "params": {"duration": 1.0}})
    chunk["actions"].append({"type": "gripper_open", "params": {}})

    return chunk

# --- Main Test ---
if __name__ == "__main__":
    if start_ursim():
        print("You can now connect your Python scripts to the simulator at IP 127.0.0.1:63352 (or the port shown in URSim).")
        print("Example: using urx.Robot('127.0.0.1') to control the robot.")
    rob, gripper = connect_robot()
    if not rob:
        exit(1)

    chunks = load_chunks()

    # Record a test chunk
    new_chunk = record_test_chunk(rob, gripper)
    chunks.append(new_chunk)
    save_chunks(chunks)

    # Playback the chunk
    playback_chunk(rob, gripper, new_chunk)

    rob.close()
    print("Test finished, connection closed.")
