import tkinter as tk
from threading import Thread
import time
import os
from utility_functions import*

# Assuming these are provided by your external files:
# from utility_functions import * (Provides 'check' object and home_position())
# from eye_contact_detector import EyeContactDetector (Provides the Eye tracking class)
# from Action_chunks import playback_chunk, execute_action (Provides robot control functions)
# from GUI import * (Provides the EyeContactApp class)

# --- GLOBAL CONFIGURATION AND DATA STRUCTURES ---
# UPDATED SLIDES LIST to match assembly_guide.py
SLIDES = [
    {"instruction": "System Initialization Complete. Robot ready for first task.", "type": "text"},
    {"instruction": "STEP 1: Pick up the screwdriver. Press 'I'M READY' when done.",
     "type": "image", "url": "https://placehold.co/600x400/3561aa/ffffff?text=STEP%201%0A(Pick%20Up%20Tool)"},
    {"instruction": "STEP 2: Loosen the top screw. Proceed with caution.",
     "type": "image", "url": "https://placehold.co/600x400/983c3c/ffffff?text=STEP%202%0A(Screwing%20Action)"},
    {"instruction": "STEP 3: Place the clip using pliers. Robot standing by.",
     "type": "image", "url": "https://placehold.co/600x400/3c9854/ffffff?text=STEP%203%0A(Assembly%20Clip)"},
    {"instruction": "ASSEMBLY COMPLETE. Robot returning to home position.", "type": "text"},
]

TOOLS = ["screw_driver"]
n = 0  # Tool index counter


# --- Robot sequential execution (MAIN LOGIC) ---
# NOTE: 'app', 'check', and 'eye_detector' must be available in the global scope

def run_robot_sequence():
    global n
    print("Robot sequence thread started.")

    # 1. CONNECT ROBOT
    # NOTE: connect_robot() is imported from robot_utils.py
    rob, gripper = connect_robot()
    if not rob:
        print("Robot connection failed. Sequence aborted.")
        return

    print(f"Waiting 5 seconds for camera calibration...")
    time.sleep(5)  # Calibration delay

    state = read_state()
    current_slide_id = state.get("current_slide_id", 0)

    # 2. INITIAL CHECKS
    # NOTE: check.home_position() relies on the external 'utility_functions' import.
    if check.home_position() == True:
        print("Initial robot home position check passed.")
        update_robot_state(key="is_running", value=True)
    else:
        print("Robot not in home position. Sequence aborted.")
        rob.close()
        return

    # Set initial state to signal robot is running and ready for step 1
    update_robot_state(slide_id=0, key="human_input_ready", value=False)

    # 3. MAIN EXECUTION LOOP
    while current_slide_id < len(SLIDES):
        slide = SLIDES[current_slide_id]
        # UPDATED: Now using 'instruction' key for logging
        print(f"\n--- Executing Slide {current_slide_id}/{len(SLIDES) - 1}: {slide['instruction']} ---")

        # Must access global instances 'app'
        global app

        # --- WAIT FOR HUMAN INPUT (For slides 1 through N-1) ---
        # Slides 0 and len(SLIDES)-1 are auto-progressing/terminal slides
        if 0 < current_slide_id < len(SLIDES) - 1:
            print("Awaiting human input ('I'M READY' button press on Streamlit GUI)...")
            while not read_state().get("human_input_ready", False):
                time.sleep(0.5)
            print("Human ready signal received. Proceeding with robot action.")

        # --- ROBOT ACTION EXECUTION ---

        # Case 1: Initial (Slide 0) - Auto-progressing slide
        if current_slide_id == 0:
            # e.g., move to tool prep area without requiring human input
            chunk = get_chunk("central_tool_location")
            playback_chunk(rob, gripper, chunk)
            current_slide_id += 1
            # Advance slide and reset button for the next step (Slide 1)
            update_robot_state(current_slide_id, clear_human_input=True)
            continue

        # Case 2: Standard Assembly Step (Slides 1 through N-2)
        if 0 < current_slide_id < len(SLIDES) - 1:

            tool_name = TOOLS[n % len(TOOLS)]

            # Step 1: Execute Robot Action
            chunk = get_chunk("central_tool_location")  # This chunk represents the start of the action
            playback_chunk(rob, gripper, chunk)

            # Sensor check - Note: This relies on the external EyeContactApp GUI loop being fast
            zone_value, proximity = app.update_frame()

            if zone_value == "" and proximity == "":

                # Sub-steps for tool/part interaction
                chunk = get_chunk(f"go_{tool_name}")
                playback_chunk(rob, gripper, chunk)

                chunk = get_chunk("go_tool_square")
                playback_chunk(rob, gripper, chunk)

                chunk = get_chunk("go_tool_view")
                playback_chunk(rob, gripper, chunk)

                # Check 2: Re-check sensor state after robot action
                zone_value, proximity = app.update_frame()

                if zone_value == "" and proximity == "":  # and wrist camera  == True

                    # Finalize tool usage / drop-off
                    chunk = get_chunk(f"go_tool_cup")
                    playback_chunk(rob, gripper, chunk)
                    chunk = get_chunk(f"go_{tool_name}")
                    playback_chunk(rob, gripper, chunk)

                    current_slide_id += 1
                    n = n + 1  # Next tool/cycle
                else:
                    print("ERROR: Sensor check failed post-action. Stopping sequence.")
                    break
            else:
                print("ERROR: Sensor check failed pre-action. Stopping sequence.")
                break

        # Case 3: Final Slide (len(SLIDES) - 1)
        if current_slide_id == len(SLIDES) - 1:
            break

        # Final state update for the loop iteration
        # This update ensures the Streamlit app moves to the next screen after the robot action is complete.
        update_robot_state(current_slide_id, clear_human_input=False)
        time.sleep(0.5)

    # 4. FINAL ACTION AND CLEANUP
    print("\n\n***************************************************")
    print("*** Sequence completed or aborted. Executing finish action. ***")

    # Execute the final "go_home" chunk
    go_home_chunk = get_chunk("go_home")
    if isinstance(go_home_chunk, dict):  # Check if the chunk was successfully retrieved
        playback_chunk(rob, gripper, go_home_chunk)
    else:
        print("WARNING: 'go_home' chunk not found or invalid. Robot position may be unsafe.")

    print("***************************************************\n\n")

    # Final state update to show completion
    update_robot_state(slide_id=len(SLIDES) - 1, key="is_running", value=False, clear_human_input=False)

    # Close the robot connection
    rob.close()
    print("Robot connection closed.")


# --- Main Application Entry Point ---

if __name__ == "__main__":
    # --- PRE-RUN SETUP (Placeholder definitions for external dependencies) ---
    # These placeholders allow the script to be run independently if your external
    # files (Check, EyeContactApp, etc.) are not available.

    class Placeholder:
        def home_position(self): return True

        def update_frame(self): return "", ""


    class PlaceholderApp:
        def __init__(self, root): pass

        def update_frame(self): return "", ""


    class PlaceholderDetector:
        pass


    # Initialize global instances, falling back to placeholders if the real classes aren't imported
    try:
        # Check if the real imported classes exist
        from GUI import EyeContactApp
        from utility_functions import Check
        from eye_contact_detector import EyeContactDetector
        from Action_chunks import playback_chunk, execute_action

        global app
        global eye_detector
        global check
        eye_detector = EyeContactDetector()
        check = Check()
    except ImportError:
        print("WARNING: External robot/GUI modules not found. Using Placeholder classes for execution flow.")
        # Fallback to placeholders
        global app
        global eye_detector
        global check
        app = PlaceholderApp(tk.Tk())
        eye_detector = PlaceholderDetector()
        check = Placeholder()
    except NameError:
        print("WARNING: External robot/GUI modules not found. Using Placeholder classes for execution flow.")
        # Fallback to placeholders if imported but not defined as global
        global app
        global eye_detector
        global check
        app = PlaceholderApp(tk.Tk())
        eye_detector = PlaceholderDetector()
        check = Placeholder()

    initialize_state()
    load_action_chunks()  # Load data from the external file

    # --- 1. Initialize External Components (Tkinter) ---
    root = tk.Tk()

    # Initialize the app with the root (essential for the GUI to work)
    try:
        app = EyeContactApp(root)
    except NameError:
        # Re-initialize app in case it's a placeholder
        app = PlaceholderApp(root)

    # --- 2. Start the robot logic in a separate thread ---
    robot_thread = Thread(target=run_robot_sequence)
    robot_thread.daemon = True
    robot_thread.start()

    # --- 3. Start the Tkinter main loop (this runs the GUI and blocks the main thread) ---
    root.mainloop()

    # Clean up the state file after exit
    if os.path.exists(STATE_FILE):
        os.remove(STATE_FILE)
    print("Application closed.")
