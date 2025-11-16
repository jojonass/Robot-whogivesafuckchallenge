import tkinter as tk
from threading import Thread
import time
import os

# --- External imports (adjust according to your file structure) ---
from gaze_tracking import *
from utility_functions import *
from eye_contact_detector import EyeContactDetector
from action_chunks import playback_chunk, execute_action, DEFAULT_ACCEL, DEFAULT_VEL
from gripper_trigger import ObjectDetectionTrigger
from Slides import SLIDES


# The three tools we use in the intermediate sequence
TOOLS = ["M4", "M5", "M3"]


# --- Robot Control Logic ---
def run_robot_sequence():
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

        # start Streamlit slide GUI after home check
        print("Starting Streamlit slides GUI after home position confirmed...")
        import subprocess
        subprocess.Popen(["streamlit", "run", "Slides.py"])
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
        print("Waiting for human to press 'I'M READY'...")
        while not read_state().get("human_input_ready", False):
            time.sleep(0.5)
        print("Human ready signal received. Proceeding with robot action.")
        update_robot_state(0, clear_human_input=True)

        # --- ROBOT ACTION EXECUTION ---
        if current_slide_id == 0:
            chunk = get_chunk("central_tool_location")
            playback_chunk(rob, gripper, chunk)
            current_slide_id += 1
            update_robot_state(current_slide_id, clear_human_input=True)
            continue

        if 0 < current_slide_id < len(SLIDES) - 1:
            # Intermediate slides: per slide, handle exactly ONE tool.
            #
            # Slide text (from SLIDES) is assumed to do:
            #   - "Use this screw" (before user presses I'M READY)
            #   - Then robot performs:
            #       2. Move to point A (tool-specific)
            #       3. Pick up tool at A → close gripper
            #       4. Move to point B (common)
            #       5. Put down tool at B → open gripper
            #       6. Move to point C (common)
            #   - Slide then explains: "When finished with tool, put it to Point C"
            #   - Robot then:
            #       7. Waits for gaze + detection
            #       8. Move to point D (dynamic)
            #       9. Close gripper at D
            #      10. Move back to point C
            #      11. Move back to point A
            #      12. Open gripper at A
            #
            # Tool order is fixed: slide1→M4, slide2→M5, slide3→M3
            tools_sequence = ["M4", "M5", "M3"]

            # Map current intermediate slide to a tool index.
            #   slide 0: initial (already handled above)
            #   slide 1: M4
            #   slide 2: M5
            #   slide 3: M3
            tool_index = current_slide_id - 1
            if tool_index < 0 or tool_index >= len(tools_sequence):
                print(f"[WARN] No tool mapped for slide {current_slide_id}. Skipping tool logic.")
                current_slide_id += 1
                continue

            tool_name = tools_sequence[tool_index]
            tool_lower = tool_name.lower()

            # Chunk names:
            chunk_A_name = f"tool_location_{tool_lower}"  # point A (tool-specific)
            chunk_B_name = "go_tool_square"               # point B (common)
            chunk_C_name = "go_tool_view"                 # point C (common)

            print(f"\n=== Intermediate slide {current_slide_id}: Tool {tool_name} ===")
            slide_success = True

            # --- 2. Move to point A (tool-specific) ---
            chunk_A = get_chunk(chunk_A_name)
            if not isinstance(chunk_A, dict):
                print(f"[ERROR] Chunk '{chunk_A_name}' not found or invalid. Aborting slide.")
                slide_success = False
            else:
                print(f"[{tool_name}] Moving to point A via chunk '{chunk_A_name}'.")
                playback_chunk(rob, gripper, chunk_A)

            if not slide_success:
                break

            # --- 3. Pick up tool at A (close gripper) ---
            print(f"[{tool_name}] Closing gripper at point A (pick up tool).")
            gripper.close_gripper()

            # --- 4. Move to point B (common) ---
            chunk_B = get_chunk(chunk_B_name)
            if not isinstance(chunk_B, dict):
                print(f"[ERROR] Chunk '{chunk_B_name}' not found or invalid. Aborting slide.")
                slide_success = False
            else:
                print(f"[{tool_name}] Moving to point B via chunk '{chunk_B_name}'.")
                playback_chunk(rob, gripper, chunk_B)

            if not slide_success:
                break

            # --- 5. Put down tool at B (open gripper) ---
            print(f"[{tool_name}] Opening gripper at point B (put down tool).")
            gripper.open_gripper()

            # --- 6. Move to point C (common) ---
            chunk_C = get_chunk(chunk_C_name)
            if not isinstance(chunk_C, dict):
                print(f"[ERROR] Chunk '{chunk_C_name}' not found or invalid. Aborting slide.")
                slide_success = False
            else:
                print(f"[{tool_name}] Moving to point C via chunk '{chunk_C_name}'.")
                playback_chunk(rob, gripper, chunk_C)

            if not slide_success:
                break

            # At this point:
            #   - Tool is lying at B
            #   - Robot is at C
            #   - Slide (in Streamlit) should say:
            #       "When finished with tool, put it to Point C"
            # The user works with the tool, then puts it down at C.

            # --- 7. Wait for valid eye gaze confirmation AND object detection at C ---
            print(f"[{tool_name}] Waiting for user to finish and put tool to point C...")
            max_wait_sec = 120.0  # simple safety timeout
            wait_start = time.time()

            pose_D = None
            while True:
                if time.time() - wait_start > max_wait_sec:
                    print(f"[{tool_name}] Timeout waiting for gaze + detection. Aborting slide.")
                    slide_success = False
                    break

                # Gaze detection
                frame, eye_contact, zone_value, _ = app.detector.detect_eye_contact(app.active_cam)
                gaze_memory.update(eye_contact, zone_value)

                gaze_ok = (zone_value == "left" or gaze_memory.recently_valid("left"))

                if not gaze_ok:
                    time.sleep(0.3)
                    continue

                # Only if gaze is OK, run object detection with calibrated pose computation.
                base_tcp_pose = rob.getl()  # [x, y, z, rx, ry, rz] in meters at point C
                detector = ObjectDetectionTrigger(
                    cam_index=1,
                    stability_time=1.5,
                    calib_file="camera_scales.npz",
                    base_tcp_pose=base_tcp_pose,
                    visualize=False,
                    timeout=5.0,
                )

                detection_ok = bool(detector)
                if detection_ok:
                    # Try to obtain dynamic target pose D from detector
                    if hasattr(detector, "get_target_pose"):
                        pose_D = detector.get_target_pose()
                    else:
                        pose_D = getattr(detector, "target_pose", None)

                    if pose_D is not None:
                        print(f"[{tool_name}] Gaze + detection gate passed at C.")
                        break
                    else:
                        print(f"[{tool_name}] Detector triggered but no target pose D available.")
                else:
                    print(f"[{tool_name}] Detection not stable yet, retrying gaze/detection...")

                time.sleep(0.3)

            if not slide_success:
                break

            if pose_D is None:
                print(
                    f"[{tool_name}] WARNING: No target pose D from detector. Cannot move to D safely. Aborting slide."
                )
                slide_success = False
                break

            # --- 8. Move to point D (dynamic, from object detector) ---
            print(f"[{tool_name}] Moving to dynamic point D from detector.")
            try:
                rob.movel(pose_D, acc=DEFAULT_ACCEL, vel=DEFAULT_VEL)
            except Exception as e:
                print(f"[{tool_name}] ERROR during movel to point D: {e}")
                slide_success = False
                break

            # --- 9. Close gripper at D (pick from D) ---
            print(f"[{tool_name}] Closing gripper at point D (pick object at D).")
            gripper.close_gripper()

            # --- 10. Move back to point C ---
            print(f"[{tool_name}] Returning to point C via chunk '{chunk_C_name}'.")
            playback_chunk(rob, gripper, chunk_C)

            # --- 11. Move back to point A (tool’s original location) ---
            print(f"[{tool_name}] Returning to point A via chunk '{chunk_A_name}'.")
            playback_chunk(rob, gripper, chunk_A)

            # --- 12. Open gripper (put it back at A) ---
            print(f"[{tool_name}] Opening gripper at point A (return tool).")
            gripper.open_gripper()

            if slide_success:
                print(f"[{tool_name}] Full A→B→C (user) →D→C→A cycle completed for this slide.")
                current_slide_id += 1
            else:
                print(f"[{tool_name}] Slide failed; aborting sequence.")
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
