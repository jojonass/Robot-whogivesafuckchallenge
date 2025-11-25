import cv2
import time
import threading
import URBasic
import robotiq_gripper
from New_instructions import*
from gaze_manager import GazeWorker, GazeMemory, wait_for_gaze_command
from object_detection_trigger import ObjectDetectionTrigger



# Gaze command zone
GAZE_COMMAND_ZONE = 'Right'
GUI_COMMAND_ZONE = 'Left'

# Shared state between robot thread and GUI
status = {
    "current_slide": 0,   # Slide index to show
    "wait_for_gaze": False,
     "wait_for_detection": False,
    "detection_result": None
}
host = "192.168.0.9"  # Adjust to match your robot / simulator IP
#  host '192.168.1.10'
acc = 1.8
vel = 1.8

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
    pose_central_tool = [-0.249, 0.348, 0.355,  - 3.148, 0.028, 0.022]
    pose_A_approach = pose_central_tool
    pose_A_grasp = [-0.257, 0.433, 0.136, - 3.148, 0.028, 0.022]
    pose_B_approach = pose_central_tool  # Step 2
    pose_B_grasp = [0.264, -0.382, 0.155, - 3.148, 0.028, 0.022]  # Step 2
    pose_C_approach = pose_central_tool  # Step 3
    pose_C_grasp = [0.248, -0.206, 0.153,- 3.148, 0.028, 0.022]  # Step 3
    pose_tool_square_approach = [-0.336, 0.196, 0.377, - 3.148, 0.028, 0.022]
    pose_tool_square_place = [-0.434, 0.263, 0.136, - 3.148, 0.028, 0.022]
    home_joints = [-0.380, -0.149, 0.3, - 3.148, 0.028, 0.022]

    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(host, 63352)
    print("Activating gripper...")
    gripper.activate()

    # -----------------------------
    # Step 1: Show Slide 1, wait for gaze and then play first instruction

    # Step 1: Show Slide 1, wait for gaze and then play first instruction
    step_counter += 1
    slide_counter = 1

    print(f"[Step {step_counter}] Showing Slide 1: Look at robot")  # Step 1
    status["current_slide"] = slide_counter  # Show slide 1
    status["wait_for_gaze"] = True
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    status["wait_for_gaze"] = False
    time.sleep(1.0)

    tools = [
        {"name": "Tool M4", "approach": pose_A_approach, "grasp": pose_A_grasp},
        #{"name": "Tool M5", "approach": pose_B_approach, "grasp": pose_B_grasp},
        #{"name": "Tool M3", "approach": pose_C_approach, "grasp": pose_C_grasp},
    ]

    for i, tool in enumerate(tools, start=1):
        print(f"\n=== Iteration {i}: Handling {tool['name']} ===")

        # Step 2: move to central tool
        robot.movel(pose=pose_central_tool, a=acc, v=vel)

        # Step 2a: combined slide
        step_counter += 1
        slide_counter += 1
        status["current_slide"] = slide_counter
        print(f"[Slide] Pick up part for {tool['name']}, then look at robot for tool")
        robot.movel(pose=tool["approach"], a=acc, v=vel)

        # Steps 3-6: robot moves while human picks part
        step_counter += 1
        print(f"[Step {step_counter}] Moving to approach for {tool['name']}")
        robot.movel(pose=tool["approach"], a=acc, v=vel)

        step_counter += 1
        print(f"[Step {step_counter}] Moving to grasp {tool['name']}")
        robot.movel(pose=tool["grasp"], a=acc, v=vel)
        gripper.move_and_wait_for_pos(255, 255, 255)

        step_counter += 1
        print(f"[Step {step_counter}] Retreating from {tool['name']} grasp")
        robot.movel(pose=tool["approach"], a=acc, v=vel)

        step_counter += 1
        print(f"[Step {step_counter}] Moving to tool square approach")
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)



        # Step 7: gaze check for tool placement
        original_threshold = worker.gaze_memory.hold_threshold
        worker.gaze_memory.hold_threshold = 2.0 # seconds
        step_counter += 1
        status["wait_for_gaze"] = True
        wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
        status["wait_for_gaze"] = False
        worker.gaze_memory.hold_threshold = original_threshold

        print(f"[Step {step_counter}] Placing {tool['name']}")
        robot.movel(pose=pose_tool_square_place, a=acc, v=vel)
        gripper.move_and_wait_for_pos(0, 255, 255)
        slide_counter += 1
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
        status["current_slide"] = slide_counter
        time.sleep(12)  # ensures tool is picked up by human before next iteration

        robot.movel(pose= pose_tool_square_place , a=acc, v=vel)
        gripper.move_and_wait_for_pos(255, 255, 255)
        robot.movel(pose= pose_tool_square_approach, a=acc, v=vel)
        robot.movel(pose=pose_A_approach, a=acc, v=vel)
        robot.movel(pose= pose_A_grasp, a=acc, v=vel)
        gripper.move_and_wait_for_pos(0, 255, 255)


        step_counter += 1
        # Include in loop when ready
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
        
        print("Robot requesting object detection")
        status["wait_for_detection"] = True
        status["detection_result"] = None
        # Pause robot thread
        while robot_status["detection_result"] is None:
            time.sleep(0.05)
        detected_pose = robot_status["detection_result"]
        rstatus["wait_for_detection"] = False
        print("Detection returned:", detected_pose)
        
    
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
    
    
    # Step 9: Return home wait for gaze to confirm finishing

    print(f"[Step {step_counter}] Returning home")
    status["current_slide"] = len(instruction_slides) - 3

    status["wait_for_gaze"] = True
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    status["wait_for_gaze"] = False

    robot.movel(pose=home_joints, a=acc, v=vel)
    status["current_slide"] = len(instruction_slides) - 2
    status["current_slide"] = len(instruction_slides) - 1
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
        slide_viewer = SlideViewer(scale=0.97)

        robotModel = URBasic.robotModel.RobotModel()
        robot = URBasic.urScriptExt.UrScriptExt(host="192.168.0.9", robotModel=robotModel)
        robot.reset_error()

        # Show initial standby slide
        current_slide_index = 0
        img = slide_viewer.prepare_slide_image(instruction_slides[0])
        cv2.imshow(slide_viewer.window_name, img)
        cv2.waitKey(1)
        time.sleep(2)

        # Start GazeWorker
        worker = GazeWorker(cam_id=0)
        worker.gaze_memory = GazeMemory(hold_threshold=0.1, memory_window=1.0)
        worker.start()
        print("[Main] GazeWorker started. Waiting for calibration...")

        while not worker.is_calibrated and worker.running:
            time.sleep(0.1)
        print("[Main] Calibration complete.")

        # Start robot routine in a separate thread
        robot_thread = threading.Thread(target=ExampleurScript, args=(worker, status))
        robot_thread.start()

        # Main loop: update slides dynamically
        current_slide_index = 0
        while robot_thread.is_alive():

            # Start from robot-provided slide index
            slide_index = status["current_slide"]

            #  Check detection request
            if status.get("wait_for_detection"):
                print("[Main] Running object detection in main loop...")
                detected_pose = wait_for_detection_and_get_pose(robot)  # pass robot object
                status["detection_result"] = detected_pose
                status["wait_for_detection"] = False  # reset the flag
                print("[Main] Detection done:", detected_pose)
                continue

            # Only re-render if slide changed
            if slide_index != current_slide_index:
                img = slide_viewer.prepare_slide_image(instruction_slides[slide_index])
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

