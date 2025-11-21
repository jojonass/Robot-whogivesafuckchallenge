import URBasic
import time
# NOTE: ObjectDetectionTrigger is assumed to handle camera logic separate from GazeWorker
from object_detection_trigger import ObjectDetectionTrigger
from New_instructions import *
import robotiq_gripper
#### initiate slides
viewer = SlideViewer(scale=0.9)


# --- GAZE CONTROL IMPORTS & CONFIGURATION ---
from gaze_manager import GazeWorker, wait_for_gaze_command, CAMERA_ID
# Define the gaze command required to advance a step (e.g., look "Right")
GAZE_COMMAND_ZONE = "Right"
# --------------------------------------------

host = "192.168.0.9"  # Adjust to match your robot / simulator IP
acc = 0.1
vel = 0.1

# ----------------------------------------------------------------------
# Placeholder helper functions for object detection
# ----------------------------------------------------------------------

def wait_for_detection_and_get_pose(robot):
    base_tcp_pose = robot.getl()
    print(f"[Robot] Base TCP for detection: {base_tcp_pose}")

    detector = ObjectDetectionTrigger(
        cam_index=0,
        stability_time=1.5,
        calib_file="camera_scales.npz",
        base_tcp_pose=base_tcp_pose,
        timeout=5.0,
    )

    # In a real scenario, you'd replace this with the actual detected pose.
    # For now, we simulate the detection returning the base pose.
    detected_pose = detector.get_target_pose()
    if detected_pose is not None:
        return detected_pose
    else:
        print("[Robot] No stable detection, falling back to base TCP pose as D.")
        return base_tcp_pose


# ----------------------------------------------------------------------
# Main script logic: Now accepts the GazeWorker instance
# Gripper initialization moved inside to control sequence.
# ----------------------------------------------------------------------
def ExampleurScript(worker: GazeWorker):
    robotModel = URBasic.robotModel.RobotModel()  # Step 0
    robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModel)  # Step 0
    robot.reset_error()  # Step 0

    viewer.show_slide_blocking_robot(instruction_slides[1])

    step_counter = 0

    # -----------------------------
    # STEP 1: Wait for Gaze Calibration
    # -----------------------------
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for Gaze Tracker Calibration...")  # Step 1
    while not worker.is_calibrated:  # Step 1
        if not worker.running:  # Step 1
            print(f"[Step {step_counter}] Worker stopped before calibration. Exiting.")  # Step 1
            return  # Step 1
        time.sleep(1)  # Step 1
    print(f"[Step {step_counter}] Gaze Calibration complete.")  # Step 1

    # -----------------------------
    # STEP 2: Gripper Initialization
    # -----------------------------
    step_counter += 1
    print(f"[Step {step_counter}] Creating and activating gripper...")  # Step 2
    #gripper = robotiq_gripper.RobotiqGripper()  # Step 2
    #gripper.connect(host, 63352)  # Step 2
    #gripper.activate()  # Step 2
    print(f"[Step {step_counter}] Gripper activated.")  # Step 2

    # -----------------------------
    # STEP 3: Pose definitions
    # -----------------------------
    step_counter += 1
    print(f"[Step {step_counter}] Defining robot poses for central tool, approaches, grasps, and tool square")  # Step 3
    pose_central_tool = [0.244, -0.32, 0.3, -3.122, -0.03, -0.123]  # Step 3
    pose_A_approach = pose_central_tool  # Step 3
    pose_A_grasp = [0.25, -0.315, 0.152, -3.122, -0.03, -0.123]  # Step 3
    pose_B_approach = pose_central_tool  # Step 3
    pose_B_grasp = [0.264, -0.382, 0.155, -3.122, -0.03, -0.123]  # Step 3
    pose_C_approach = pose_central_tool  # Step 3
    pose_C_grasp = [0.248, -0.206, 0.153, -3.122, -0.03, -0.123]  # Step 3
    pose_tool_square_approach = [-0.028, -0.371, 0.282, -3.122, -0.03, -0.12]  # Step 3
    pose_tool_square_place = [-0.041, -0.393, 0.155, -3.122, -0.03, -0.12]  # Step 3
    home_joints = [0.06, -0.322, 0.405, -3.122, -0.03, -0.123]  # Step 3

    # -----------------------------
    # STEP 4: Wait for User READY
    # -----------------------------
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for user to click 'I'M READY' (GUI)...")  # Step 4
    viewer.show_slide_human(instruction_slides[1], require_button=True)  # Step 4
    worker.set_active(True)  # Step 4
    print(f"[Step {step_counter}] User ready confirmed. Gaze worker activated.")  # Step 4

    # =======================
    # SEQUENCE 1: M4 assembly
    # =======================
    # STEP 5: Show slide and wait for gaze
    step_counter += 1
    print(f"[Step {step_counter}] Showing Slide 1 instructions (M4: big gear, small gear)")  # Step 5
    viewer.show_slide_human(instruction_slides[2], require_button=False)   # Step 5
    print(f"[Step {step_counter}] Waiting for human gaze at GUI before picking M4")  # Step 5
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)  # Step 5

    # STEP 6: Move to approach
    step_counter += 1
    print(f"[Step {step_counter}] Moving to approach M4 (pose_A_approach)")  # Step 6
    robot.movel(pose=pose_A_approach, a=acc, v=vel)  # Step 6

    # STEP 7: Move to grasp
    step_counter += 1
    print(f"[Step {step_counter}] Moving to grasp M4 (pose_A_grasp)")  # Step 7
    robot.movel(pose=pose_A_grasp, a=acc, v=vel)  # Step 7

    # STEP 8: Close gripper
    step_counter += 1
    print(f"[Step {step_counter}] Closing gripper to pick M4 tool")  # Step 8
    #gripper.move_and_wait_for_pos(255, 255, 255)  # Step 8

    # STEP 9: Retreat
    step_counter += 1
    print(f"[Step {step_counter}] Retreating from M4 (pose_A_approach)")  # Step 9
    robot.movel(pose=pose_A_approach, a=0.3, v=vel)  # Step 9

    # STEP 10: Move to tool square approach
    step_counter += 1
    print(f"[Step {step_counter}] Moving to tool square approach")  # Step 10
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)  # Step 10

    # STEP 11: Move to tool square place with gaze check
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before moving to place M4")
    wait_for_gaze_command(worker, "Left")
    print(f"[Step {step_counter}] Moving to tool square place")
    robot.movel(pose=pose_tool_square_place, a=0.3, v=vel)


    # STEP 12: Human gaze check before placing
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before placing M4")
    wait_for_gaze_command(worker, "Left")
    #gripper.move_and_wait_for_pos(0, 255, 255)
    robot.movel(pose=pose_tool_square_approach, a=0.3, v=vel)

    # STEP 13: Move to Pose D (with optional gaze confirmation)
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before moving to Pose D (M4)")  # Step 13

    # --- Gaze memory confirmation before moving ---
    wait_for_gaze_command(worker, "Left")  # Step 13: human must look at robot

    print(f"[Step {step_counter}] Moving to Pose D (M4) - optional detector commented")  # Step 13
    target_pose_D = pose_tool_square_place  # Step 13

    # Optional tool detection logic (currently commented)
    # if detector.check_pose_stable(target_pose_D):
    #     print(f"[Step {step_counter}] Pose D detected OK")
    robot.movel(pose=target_pose_D, a=0.3, v=vel)  # Step 13


    # STEP 14: Close gripper at D
    step_counter += 1
    print(f"[Step {step_counter}] Closing gripper at Pose D (M4)")  # Step 14
    #gripper.move_and_wait_for_pos(255, 255, 255)  # Step 14

    # STEP 15: Return to point A with object
    step_counter += 1
    print(f"[Step {step_counter}] Returning to point A with object")  # Step 15
    robot.movel(pose=pose_A_approach, a=acc, v=vel)  # Step 15
    robot.movel(pose=pose_A_grasp, a=0.3, v=vel)  # Step 15

    # STEP 16: Open gripper to return tool
    step_counter += 1
    print(f"[Step {step_counter}] Opening gripper to return M4 tool")  # Step 16
    #gripper.move_and_wait_for_pos(0, 255, 255)  # Step 16

    # =======================
    # SEQUENCE 2: M5 assembly
    # =======================

    # STEP 17: Show slide and wait for gaze at GUI
    step_counter += 1
    print(f"[Step {step_counter}] Showing Slide 2 instructions (M5: hand)")  # Step 17
    viewer.show_slide_human(instruction_slides[3], require_button=False)  # Step 17
    print(f"[Step {step_counter}] Waiting for human gaze at GUI before picking M5")  # Step 17
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)  # Step 17

    # STEP 18: Move to approach M5
    step_counter += 1
    print(f"[Step {step_counter}] Moving to approach M5 (pose_B_approach)")  # Step 18
    robot.movel(pose=pose_B_approach, a=acc, v=vel)  # Step 18

    # STEP 19: Move to grasp M5
    step_counter += 1
    print(f"[Step {step_counter}] Moving to grasp M5 (pose_B_grasp)")  # Step 19
    robot.movel(pose=pose_B_grasp, a=acc, v=vel)  # Step 19

    # STEP 20: Close gripper to pick M5
    step_counter += 1
    print(f"[Step {step_counter}] Closing gripper to pick M5")  # Step 20
    gripper.move_and_wait_for_pos(255, 255, 255)  # Step 20

    # STEP 21: Retreat from M5
    step_counter += 1
    print(f"[Step {step_counter}] Retreating from M5")  # Step 21
    robot.movel(pose=pose_B_approach, a=0.3, v=vel)  # Step 21

    # STEP 22: Move to tool square approach
    step_counter += 1
    print(f"[Step {step_counter}] Moving to tool square approach")  # Step 22
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)  # Step 22

    # STEP 23: Move to tool square place with gaze check
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before moving to place M5")  # Step 23
    wait_for_gaze_command(worker, "Left")  # gaze confirmation
    print(f"[Step {step_counter}] Moving to tool square place")  # Step 23
    robot.movel(pose=pose_tool_square_place, a=0.3, v=vel)  # Step 23

    # STEP 24: Human gaze check before placing M5
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before placing M5")  # Step 24
    wait_for_gaze_command(worker, "Left")  # Step 24: gaze confirmation
    gripper.move_and_wait_for_pos(0, 255, 255)  # Step 24: release M5
    robot.movel(pose=pose_tool_square_approach, a=0.3, v=vel)  # Step 24: retreat

    # =======================
    # SEQUENCE 3: M3 assembly
    # =======================
    # STEP 25: Show slide and wait for gaze at GUI
    step_counter += 1
    print(f"[Step {step_counter}] Showing Slide 3 instructions (M3: gadget)")  # Step 25
    viewer.show_slide_human(instruction_slides[4], require_button=False)  # Step 25
    print(f"[Step {step_counter}] Waiting for human gaze at GUI before picking M3")  # Step 25
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)  # Step 25

    # STEP 26: Move to approach M3
    step_counter += 1
    print(f"[Step {step_counter}] Moving to approach M3 (pose_C_approach)")  # Step 26
    robot.movel(pose=pose_C_approach, a=acc, v=vel)  # Step 26

    # STEP 27: Move to grasp M3
    step_counter += 1
    print(f"[Step {step_counter}] Moving to grasp M3 (pose_C_grasp)")  # Step 27
    robot.movel(pose=pose_C_grasp, a=acc, v=vel)  # Step 27

    # STEP 28: Close gripper to pick M3
    step_counter += 1
    print(f"[Step {step_counter}] Closing gripper to pick M3")  # Step 28
    gripper.move_and_wait_for_pos(255, 255, 255)  # Step 28

    # STEP 29: Retreat from M3
    step_counter += 1
    print(f"[Step {step_counter}] Retreating from M3")  # Step 29
    robot.movel(pose=pose_C_approach, a=0.3, v=vel)  # Step 29

    # STEP 30: Move to tool square approach
    step_counter += 1
    print(f"[Step {step_counter}] Moving to tool square approach")  # Step 30
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)  # Step 30

    # STEP 31: Move to tool square place with gaze check
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before moving to place M3")  # Step 31
    wait_for_gaze_command(worker, "Left")  # gaze confirmation
    print(f"[Step {step_counter}] Moving to tool square place")  # Step 31
    robot.movel(pose=pose_tool_square_place, a=0.3, v=vel)  # Step 31

    # STEP 32: Human gaze check before placing M3
    step_counter += 1
    print(f"[Step {step_counter}] Waiting for human gaze at robot before placing M3")  # Step 32
    wait_for_gaze_command(worker, "Left")  # Step 32: gaze confirmation
    gripper.move_and_wait_for_pos(0, 255, 255)  # Step 32: release M3
    robot.movel(pose=pose_tool_square_approach, a=0.3, v=vel)  # Step 32: retreat

    # -----------------------------
    # FINAL STEP: Return home
    # -----------------------------
    step_counter += 1
    print(f"[Step {step_counter}] Returning to home position")  # Step 33
    robot.movel(pose=home_joints, a=acc, v=vel)  # Step 33

    step_counter += 1
    print(f"[Step {step_counter}] Deactivating gaze worker and showing thank you slide")  # Step 34
    worker.set_active(False)  # Step 34
    viewer.show_slide_human(instruction_slides[5], require_button=False)   # Step 34



# -----------------------------
if __name__ == '__main__':
    # 1️⃣ Start Gaze Worker
    gaze_worker = GazeWorker(cam_id=0)
    gaze_worker.start()

    try:
        print("\n--- Gaze-Controlled Robot Routine Active ---")
        print(f"Gaze command to advance step is: '{GAZE_COMMAND_ZONE}'")

        # 2️⃣ Start main robot routine
        ExampleurScript(gaze_worker)

    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")

    finally:
        # 3️⃣ Cleanup Gaze Worker
        print("\nShutting down Gaze Worker...")
        gaze_worker.running = False
        gaze_worker.join()
        print("Gaze Worker stopped.")