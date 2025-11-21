import tkinter as tk
from threading import Thread
import time
import os

# --- External imports (adjust according to your file structure) ---
from gaze_tracking import *
from utility_functions import *
from eye_contact_detector import EyeContactDetector
from action_chunks import playback_chunk, execute_action
from gripper_trigger import *
from Slides import SLIDES


TOOLS = ["screw_driver"]
n_tool = len(TOOLS) - 1  # Tool index counter


# --- Robot Control Logic ---
def run_robot_sequence():
    global n_tool
    print("Robot sequence thread started.")

    # 1. CONNECT ROBOT
    rob, gripper = connect_robot()
    if not rob:
        print("Robot connection failed. Sequence aborted.")
        return

    print(f"Waiting 5 seconds for camera calibration...")
    time.sleep(5)  # Calibration delay

    state = read_state()
    current_slide_id = state.get("current_slide_id", 0)

    # 2. INITIAL CHECKS
    home_data = load_home_position()
    if home_data is None:
        print("No home position recorded. Recording current position as home.")
        record_home_position(rob, gripper)

    if is_at_home(rob):
        print("Robot is at home position. Ready to start.")
        update_robot_state(key="is_running", value=True)

        # --- CHANGED: start Streamlit slide GUI after home check ---
        print("Starting Streamlit slides GUI after home position confirmed...")
        import subprocess
        subprocess.Popen(["streamlit", "run", "Slides.py"])  # CHANGED
    else:
        print("Robot not at home position. Please move robot to home first.")
        rob.close()
        return

    # Initialize gaze memory
    gaze_memory = GazeMemory(hold_threshold=1.0, memory_window=10.0)

    # 3. MAIN EXECUTION LOOP
    while current_slide_id < len(SLIDES):
        slide = SLIDES[current_slide_id]
        print(f"\n--- Executing Slide {current_slide_id}/{len(SLIDES) - 1}: {slide['instruction']} ---")

        global app

        # --- WAIT FOR HUMAN INPUT (Slides 1 through N-1) ---
        print("Waiting for human to press 'I'M READY' on first slide...")
        while not read_state().get("human_input_ready", False):  # CHANGED
            time.sleep(0.5)
        print("Human ready signal received. Proceeding with robot action.")  # CHANGED
        update_robot_state(0, clear_human_input=True)  # CHANGED

        # --- ROBOT ACTION EXECUTION ---
        if current_slide_id == 0:
            chunk = get_chunk("central_tool_location")
            playback_chunk(rob, gripper, chunk)
            current_slide_id += 1
            update_robot_state(current_slide_id, clear_human_input=True)
            continue

        if 0 < current_slide_id < len(SLIDES) - 1:
            tool_name = TOOLS[n_tool]

            # Step 1: Execute Robot Action
            chunk = get_chunk("central_tool_location")
            playback_chunk(rob, gripper, chunk)

            # --- GAZE DETECTION ---
            frame, eye_contact, zone_value, _ = app.detector.detect_eye_contact(app.active_cam)
            gaze_memory.update(eye_contact, zone_value)

            if not zone_value:
                print("Zone not detected. Waiting...")
            else:
                print(f"Detected gaze zone: {zone_value}, Eye contact: {eye_contact}")

                # Pick up tool
                chunk = get_chunk(f"go_{tool_name}")
                playback_chunk(rob, gripper, chunk)
                detector = ObjectDetectionTrigger(cam_index=1, stability_time=0.3)
                if detector:
                    gripper.close_gripper()
                chunk = get_chunk("go_tool_square")
                playback_chunk(rob, gripper, chunk)

                chunk = get_chunk("go_tool_view")
                playback_chunk(rob, gripper, chunk)
                gripper.open_gripper()

                # Re-check gaze after robot action
                frame, eye_contact, zone_value, _ = app.detector.detect_eye_contact(app.active_cam)
                gaze_memory.update(eye_contact, zone_value)
                detector = ObjectDetectionTrigger(cam_index=1, stability_time=1.5)

                if (zone_value == "left" or gaze_memory.recently_valid("left")) and detector:
                    print(" Valid gaze (current or recent) confirmed — executing next action.")
                    chunk = get_chunk(f"go_tool_cup")
                    playback_chunk(rob, gripper, chunk)

                    chunk = get_chunk(f"go_{tool_name}")
                    playback_chunk(rob, gripper, chunk)

                    current_slide_id += 1
                    n_tool += 1
                else:
                    print(" Gaze zone not valid or no recent gaze — stopping sequence.")
                    break

        if current_slide_id == len(SLIDES) - 1:
            break

        update_robot_state(current_slide_id, clear_human_input=False)
        time.sleep(0.5)

    # 4. FINAL ACTION AND CLEANUP
    print("\n\n***************************************************")
    print("*** Sequence completed or aborted. Executing finish action. ***")

    go_home_chunk = get_chunk("go_home")
    if isinstance(go_home_chunk, dict):
        playback_chunk(rob, gripper, go_home_chunk)
    else:
        print("WARNING: 'go_home' chunk not found or invalid.")

    update_robot_state(slide_id=len(SLIDES) - 1, key="is_running", value=False, clear_human_input=False)
    rob.close()
    print("Robot connection closed.")


# --- Main Application Entry Point ---
if __name__ == "__main__":
    initialize_state()

    # Start Tkinter GUI
    root = tk.Tk()
    app = EyeContactApp(root)

    # Start robot sequence in a separate thread
    robot_thread = Thread(target=run_robot_sequence)
    robot_thread.daemon = True
    robot_thread.start()

    # Start Tkinter main loop
    root.mainloop()
