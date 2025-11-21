import cv2
import time
import threading
import URBasic
import robotiq_gripper
from New_instructions import SlideViewer, instruction_slides
from gaze_manager import GazeWorker, GazeMemory, wait_for_gaze_command
from object_detection_trigger import ObjectDetectionTrigger

# Gaze command zone
GAZE_COMMAND_ZONE = 'Right'
GUI_COMMAND_ZONE = 'Left'

# Shared state between robot thread and GUI
robot_status = {
    "current_slide": 0,   # Slide index to show
    "wait_for_gaze": False
}
host = "192.168.0.9"  # Adjust to match your robot / simulator IP
#  host '192.168.1.10'
acc = 0.1
vel = 0.1

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




# =========================
# Robot routine with dynamic slides
# =========================
def ExampleurScript(worker, status):

    step_counter = 0
    # Joint positions
    pose_central_tool = [-0.243, 0.278, 0.371,  - 3.080, 0.163, -0.08]
    pose_A_approach = pose_central_tool
    pose_A_grasp = [-0.24, 0.34, -0.260, - 3.080, 0.163, -0.08]
    pose_B_approach = pose_central_tool  # Step 2
    pose_B_grasp = [0.264, -0.382, 0.155, -3.122, -0.03, -0.123]  # Step 2
    pose_C_approach = pose_central_tool  # Step 3
    pose_C_grasp = [0.248, -0.206, 0.153, -3.122, -0.03, -0.123]  # Step 3
    pose_tool_square_approach = [-0.268, 0.334, 0.275, - 3.080, 0.163, -0.08]
    pose_tool_square_place = [-0.452, 0.173, 0.133, - 3.080, 0.163, -0.08]
    home_joints = [-0.380, -0.149, 0.3, - 3.080, 0.163, -0.08]

    robotModel = URBasic.robotModel.RobotModel()
    robot = URBasic.urScriptExt.UrScriptExt(host="192.168.0.9", robotModel=robotModel)
    robot.reset_error()



    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(host, 63352)
    print("Activating gripper...")
    gripper.activate()


    # -----------------------------
    # Step 1: Show Slide 1, wait for gaze and then play first instruction

    step_counter += 1
    slide_counter = 1
    print(f"[Step {step_counter}] Showing Slide 1 Look at robot")  # Step 1
    status["current_slide"] = slide_counter # Need to lok at robot
    status["wait_for_gaze"] = True
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    status["wait_for_gaze"] = False
    slide_counter += 1
    time.sleep(3.0)
    status["current_slide"] = slide_counter
    slide_counter += 1


    # Step 2:
    step_counter += 1
    print(f"[Step {step_counter}] Moving to pose_central_tool")  # Step 2
    robot.movel(pose= pose_central_tool, a=acc, v=vel)

    # Step 3:
    step_counter += 1
    print(f"[Step {step_counter}] Moving to approach M4 (pose_A_approach)")  # Step 2
    robot.movel(pose= pose_A_approach, a=acc, v=vel)  # Step

    # Step 4:
    step_counter += 1
    print(f"[Step {step_counter}] Moving to grasp M4 (pose_A_grasp)")  # Step 7
    robot.movel(pose=pose_A_grasp, a=acc, v=vel)
    gripper.move_and_wait_for_pos(255, 255, 255)

    # STEP 5:
    step_counter += 1
    print(f"[Step {step_counter}] Retreating from M4 (pose_A_approach)")  # Step 9
    robot.movel(pose=pose_A_approach, a=0.3, v=vel)

    # STEP 6 Tell human to look at robot for placing tool slide
    step_counter += 1
    print(f"[Step {step_counter}] Moving to tool square approach")  #
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)  #
    status["current_slide"] = slide_counter


    # Step 7: # Checking that person is paying attention to the tool placement
    step_counter += 1
    print(f"[Step {step_counter}] Moving to tool square place")
    status["current_slide"] = slide_counter
    status["wait_for_gaze"] = True
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    status["wait_for_gaze"] = False
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
    robot.movel(pose=pose_tool_square_place, a=acc, v=vel)
    gripper.move_and_wait_for_pos(0, 255, 255)
    slide_counter += 1
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)


   ### Step 8 for detection moke proposal

    '''

    # NEW STEP 8: Object Detection and Gaze Check Before Pick-Up (The core of your request)
    step_counter += 1
    print(f"[Step {step_counter}] Moving back to approach tool for verification.")

    # Move the robot to the camera inspection pose (pose_tool_square_approach)
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)

    # Signal the GUI that object detection is now active (for feedback/status)
    print("[Robot] Triggering combined object detection and gaze check.")
    # We introduce a new status flag for detection
    status["wait_for_detection"] = True
    status["current_slide"] = slide_counter  # Keep showing the transition slide

    Wait for Gaze to confirm user is ready for robot to proceed
    # 8a:  
    
    status["wait_for_gaze"] = True
    status["current_slide"] = slide_counter  # Keep transition slide visible
    wait_for_gaze_command(worker, _GAZE_COMMAND_ZONE)  # Wait for user to look back at Robot
    status["wait_for_gaze"] = False
    
    # 8b: Wait for successful object detection (e.g., to confirm gear is placed)
    
    print("[Robot] Waiting for object detection to confirm user task is complete.")
    detected_pose = wait_for_detection_and_get_pose(robot)
    status["wait_for_detection"] = False
    
    
    #### HMMM IDK HOW LONG of a time out do we want here or do we want them to look at tool place,
    then look back at the gui and then detected pose is activated???????

    if detected_pose is None:
        print("[Robot] ERROR: Detection failed. Halting routine.")
        # Handle error state, perhaps move to a final error slide
        status["current_slide"] = 0  # Fall back to standby/error
        return
    print("[Robot] Object detection successful. Waiting for user attention (Gaze).")


    # 8c: Robot picks up the tool/object using the detected pose
    step_counter += 1
    print(f"[Step {step_counter}] Picking up object using detected pose.")
    robot.movel(pose=detected_pose, a=acc, v=vel)
    # gripper.move_and_wait_for_pos(255, 255, 255) # Grasp the tool/gear

    # Retreat and continue the routine (e.g., move to next slide)
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
    '''

    # Step 9: Return home
    step_counter += 1
    print(f"[Step {step_counter}] Returning home")
    robot.movel(pose=home_joints, a=acc, v=vel)
    status["current_slide"] = 6
    print(f"[Step {step_counter+1}] Showing final slide")


# =========================
# Worker shutdown helper
# =========================
def stop_worker_gracefully(worker):
    if worker and worker.is_alive():
        print(" Shutting down GazeWorker...")
        worker.stop()
        worker.join()
        print(" GazeWorker stopped.")

# =========================
# Main
# =========================
if __name__ == "__main__":
    worker = None
    try:
        # Initialize SlideViewer
        slide_viewer = SlideViewer(scale=0.8)

        # Show initial standby slide
        img = slide_viewer.prepare_slide_image(instruction_slides[0])
        cv2.imshow(slide_viewer.window_name, img)
        cv2.waitKey(1)
        time.sleep(2)

        # Start GazeWorker
        worker = GazeWorker(cam_id=0)
        worker.gaze_memory = GazeMemory(hold_threshold=0.5, memory_window=3.0)
        worker.start()
        print("[Main] GazeWorker started. Waiting for calibration...")

        while not worker.is_calibrated and worker.running:
            time.sleep(0.1)
        print("[Main] Calibration complete.")

        # Start robot routine in a separate thread
        robot_thread = threading.Thread(target=ExampleurScript, args=(worker, robot_status))
        robot_thread.start()

        # Main loop: update slides dynamically
        current_slide_index = -1
        while robot_thread.is_alive():
            slide_index = robot_status["current_slide"]
            if slide_index != current_slide_index:
                img = slide_viewer.prepare_slide_image(instruction_slides[slide_index])
                if robot_status.get("wait_for_gaze", False):
                    cv2.putText(img, "Look at the robot! \n return to instructions when robot moves", (50, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow(slide_viewer.window_name, img)
                current_slide_index = slide_index

            if cv2.waitKey(50) & 0xFF == ord("q"):
                break
            time.sleep(0.05)

        robot_thread.join()

        # Keep final slide open until user closes
        while True:
            key = cv2.waitKey(100) & 0xFF
            if key == ord("q"):
                break

    finally:
        stop_worker_gracefully(worker)
        cv2.destroyAllWindows()
        print(" Program successfully shut down.")
