import tkinter as tk
from threading import Thread
import time
import random

# ----------------- MOCK SLIDES -----------------
SLIDES = [
    {"instruction": "System Initialization Complete. Robot ready for first task.", "type": "text"},
    {"instruction": "STEP 1: Pick up the screwdriver.", "type": "text"},
    {"instruction": "STEP 2: Loosen the top screw.", "type": "text"},
    {"instruction": "ASSEMBLY COMPLETE. Robot returning to home position.", "type": "text"},
]

# ----------------- MOCK ACTION CHUNKS -----------------
MOCK_CHUNKS = {
    "central_tool_location": {"name": "central_tool_location", "actions": ["Move to central tool location"]},
    "go_screw_driver": {"name": "go_screw_driver", "actions": ["Pick up screwdriver"]},
    "go_tool_square": {"name": "go_tool_square", "actions": ["Move to tool square"]},
    "go_tool_view": {"name": "go_tool_view", "actions": ["View tool"]},
    "go_tool_cup": {"name": "go_tool_cup", "actions": ["Move to cup"]},
    "go_home": {"name": "go_home", "actions": ["Return home"]},
}

# ----------------- MOCK ROBOT/GRIPPER -----------------
class MockGripper:
    def __init__(self):
        self.state = "open"
    def open_gripper(self):
        self.state = "open"
        print("Gripper opened")
    def close_gripper(self):
        self.state = "closed"
        print("Gripper closed")

def connect_robot():
    print("Mock robot connected")
    return "robot", MockGripper()

def is_at_home(rob):
    return True

def load_home_position():
    return {"x":0,"y":0,"z":0}

def record_home_position(rob, gripper):
    print("Recorded mock home position")

def get_chunk(name):
    return MOCK_CHUNKS.get(name)

def playback_chunk(rob, gripper, chunk):
    print(f"\n--- Executing chunk: {chunk['name']} ---")
    for action in chunk['actions']:
        print(f" > {action}")
        time.sleep(0.3)

# ----------------- MOCK EYE GAZE DETECTOR -----------------
class MockEyeGazeDetector:
    def detect_eye_contact(self, cam_id):
        eye_contact = random.choice([True, False])
        zone = random.choice(["left", "center", "right"]) if eye_contact else None
        return None, eye_contact, zone, None

# ----------------- MOCK OBJECT DETECTION -----------------
class ObjectDetectionTrigger:
    def __init__(self, cam_index=0, stability_time=1.5):
        self.start_t = time.time()
        self.stability_time = stability_time
    def __bool__(self):
        return (time.time() - self.start_t) > self.stability_time

# ----------------- GAZE MEMORY -----------------
class GazeMemory:
    def __init__(self, hold_threshold=1.0, memory_window=10.0):
        self.hold_threshold = hold_threshold
        self.memory_window = memory_window
        self.last_hold_time = None
        self.last_zone = None
        self._contact_start = None
        self._current_zone = None

    def update(self, eye_contact_active, zone_value):
        now = time.time()
        if eye_contact_active and zone_value:
            if self._contact_start is None or zone_value != self._current_zone:
                self._contact_start = now
                self._current_zone = zone_value
            elif now - self._contact_start >= self.hold_threshold:
                self.last_hold_time = now
                self.last_zone = zone_value
        else:
            self._contact_start = None
            self._current_zone = None

    def recently_valid(self, zone_required):
        if self.last_hold_time is None:
            return False
        if self.last_zone != zone_required:
            return False
        return (time.time() - self.last_hold_time) < self.memory_window

# ----------------- TKINTER GUI -----------------
class TestApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Sequence Test")
        self.current_slide = 0
        self.human_ready = False

        self.label = tk.Label(root, text="System Ready", font=("Helvetica", 18))
        self.label.pack(pady=10)

        self.slide_label = tk.Label(root, text="Slide: --", font=("Helvetica", 14))
        self.slide_label.pack(pady=5)
        self.update_slide_label()

        self.ready_btn = tk.Button(root, text="I'M READY / TASK COMPLETE", font=("Helvetica", 14),
                                   command=self.mark_ready)
        self.ready_btn.pack(pady=10)

    def update_slide_label(self):
        self.slide_label.config(
            text=f"Slide {self.current_slide}: {SLIDES[self.current_slide]['instruction']}"
        )
        self.root.after(500, self.update_slide_label)

    def mark_ready(self):
        print("Human pressed 'I'M READY'")
        self.human_ready = True
        self.ready_btn.config(state="disabled")


# ----------------- ROBOT SEQUENCE (NEW LOGIC) -----------------
TOOLS = ["screw_driver"]
n_tool = 0

def run_robot_sequence(app):
    global n_tool
    print("Robot sequence thread started")

    rob, gripper = connect_robot()
    gaze_memory = GazeMemory()
    front_detector = MockEyeGazeDetector()

    time.sleep(2)
    current_slide_id = 0

    # ------------- MAIN LOOP (MATCHES REAL LOGIC) -------------
    while current_slide_id < len(SLIDES):
        print(f"\n=== Slide {current_slide_id}: {SLIDES[current_slide_id]['instruction']} ===")
        app.current_slide = current_slide_id

        # --- WAIT FOR HUMAN INPUT on every slide before action ---
        print("Waiting for human input...")
        while not app.human_ready:
            time.sleep(0.3)
        print("Human ready confirmed. Executing robot action.\n")
        app.human_ready = False
        app.ready_btn.config(state="normal")

        # ------------ SLIDE 0 (intro slide) ------------
        if current_slide_id == 0:
            playback_chunk(rob, gripper, get_chunk("central_tool_location"))
            current_slide_id += 1
            continue

        # ------------ MAIN ACTION SLIDES ------------
        if 0 < current_slide_id < len(SLIDES) - 1:
            tool_name = TOOLS[n_tool]

            # Step 1: Move to tool area
            playback_chunk(rob, gripper, get_chunk("central_tool_location"))

            # Step 2: First gaze check
            _, eye_contact, zone_value, _ = front_detector.detect_eye_contact(0)
            gaze_memory.update(eye_contact, zone_value)
            print(f"Gaze: {eye_contact}, zone: {zone_value}")

            # Step 3: Use tool
            playback_chunk(rob, gripper, get_chunk(f"go_{tool_name}"))

            detector = ObjectDetectionTrigger(stability_time=0.3)
            if detector:
                gripper.close_gripper()

            playback_chunk(rob, gripper, get_chunk("go_tool_square"))
            playback_chunk(rob, gripper, get_chunk("go_tool_view"))
            gripper.open_gripper()

            # Step 4: Re-check gaze after robot action
            _, eye_contact, zone_value, _ = front_detector.detect_eye_contact(0)
            gaze_memory.update(eye_contact, zone_value)
            print(f"Re-check gaze: {eye_contact}, zone: {zone_value}")

            detector = ObjectDetectionTrigger(stability_time=1.5)

            # ACCEPT LEFT ZONE or recent left memory
            if (zone_value == "left" or gaze_memory.recently_valid("left")) and detector:
                print("Valid gaze — continuing\n")
                playback_chunk(rob, gripper, get_chunk("go_tool_cup"))
                playback_chunk(rob, gripper, get_chunk(f"go_{tool_name}"))
                n_tool += 1
                current_slide_id += 1
            else:
                print("Invalid/no gaze — STOPPING sequence.")
                break

        # ------------ END SLIDE ------------
        if current_slide_id == len(SLIDES) - 1:
            break

    print("\nSequence finished — Returning Home")
    playback_chunk(rob, gripper, get_chunk("go_home"))
    print("Robot connection closed.")


# ----------------- MAIN -----------------
if __name__ == "__main__":
    root = tk.Tk()
    app = TestApp(root)

    t = Thread(target=run_robot_sequence, args=(app,))
    t.start()

    root.mainloop()
