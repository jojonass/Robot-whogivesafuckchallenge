import cv2
import time
import threading

from pyparsing import original_text_for

import URBasic
import robotiq_gripper
from instructions_gaze import *
from gaze_manager import GazeWorker, GazeMemory, wait_for_gaze_command
from object_detection_trigger import ObjectDetectionTrigger
import random

# Gaze command zone
GAZE_COMMAND_ZONE = 'Right'
GUI_COMMAND_ZONE = 'Left'

# Shared state between robot thread and GUI
status = {
    "current_slide": SLIDE_STANDBY,  # Slide index to show
    "wait_for_gaze": False,
    "wait_for_detection": False,
    "detection_result": None
}
host = "192.168.0.9"  # Adjust to match your robot / simulator IP
#  host '192.168.1.10'
acc = 0.5
vel = 0.5

"""
blue_detector = ObjectDetectionTrigger(
    cam_index=1,
    stability_time=2.0,   # wait for 2 seconds of stability
    timeout=9999.0        # effectively wait "forever"
)
"""


def random_addition(waypoint, spread=0.05):
    """
    Generate a random addition problem wher the first number is near `center`
    and the second number is small.
    """
    a1 = waypoint[0] + random.uniform(-spread, spread)
    a2 = waypoint[1] + random.uniform(-spread, spread)
    a3 = waypoint[2] + random.uniform(-spread, spread)

    waypoint = [a1, a2, a3, - 2.364, -2.26, 0.171]

    return waypoint


# =========================
# Robot routine with dynamic slides
# =========================
def ExampleurScript(worker, status):
    step_counter = 0
    # Joint positions
    pose_central_tool = [-0.249, 0.348, 0.355, - 2.364, -2.26, 0.171]

    pose_A_approach = pose_central_tool
    pose_A_grasp = [-0.265, 0.457, 0.134, - 2.364, -2.26, 0.171]

    pose_B_approach = pose_central_tool
    pose_B_grasp = [0.264, -0.382, 0.155, - 2.364, -2.26, 0.171]

    pose_C_approach = pose_central_tool
    pose_C_grasp = [0.248, -0.206, 0.153, - 2.364, -2.26, 0.171]

    pose_tool_square_approach = [-0.336, 0.196, 0.377, - 2.364, -2.26, 0.171]
    pose_tool_square_place = [-0.445, 0.260, 0.134, - 2.364, -2.26, 0.171]

    home_joints = [-0.380, -0.149, 0.3, - 2.364, -2.26, 0.171]

    workspace_points = [
        [-0.25, 0.25, 0.35, -2.364, -2.26, 0.171],
        [-0.22, 0.22, 0.30, -2.364, -2.26, 0.171],
        [-0.20, 0.20, 0.28, -2.364, -2.26, 0.171]
    ]

    # Abrupt waypoints

    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(host, 63352)
    print("Activating gripper...")
    gripper.activate()

    # -----------------------------

    # Step 1: Show Slide 1, wait for gaze and then play first instruction
    original_threshold = worker.gaze_memory.hold_threshold

    print(f"[Step {step_counter}] Showing Slide 1: Look at robot")  # Step 1
    status["current_slide"] = SLIDE_LOOK_AT_ROBOT  # Show slide 1
    status["wait_for_gaze"] = True
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    status["wait_for_gaze"] = False
    time.sleep(1.0)

    tools = [
        {"name": "M4", "approach": pose_A_approach, "grasp": pose_A_grasp},
        {"name": "M5", "approach": pose_B_approach, "grasp": pose_B_grasp},
        {"name": "M3", "approach": pose_C_approach, "grasp": pose_C_grasp},
    ]

    for tool in enumerate(tools, start=1):
        print(f"\n=== Iteration {i}: Handling {tool['name']} ===")

        # Map tool name -> slides
        if tool["name"] == "M4":
            pickup_slide = SLIDE_PICKUP_M4
            place_slide = SLIDE_PLACE_M4
        elif tool["name"] == "M5":
            pickup_slide = SLIDE_PICKUP_M5
            place_slide = SLIDE_PLACE_M5
        elif tool["name"] == "M3":
            pickup_slide = SLIDE_PICKUP_M3
            place_slide = SLIDE_PLACE_M3
        else:
            pickup_slide = SLIDE_STANDBY
            place_slide = SLIDE_STANDBY

        # Move to central tool location
        robot.movel(pose=pose_central_tool, a=acc, v=vel)

        # Pick up tool
        status["current_slide"] = pickup_slide
        robot.movel(pose=tool["approach"], a=acc, v=vel)
        robot.movel(pose=tool["grasp"], a=acc, v=vel)
        gripper.move_and_wait_for_pos(255, 255, 255)
        robot.movel(pose=tool["approach"], a=acc, v=vel)

        # Wait for gaze check at tool_square
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
        time.sleep(1.5)

        # Gaze check for tool placement
        original_threshold = worker.gaze_memory.hold_threshold
        worker.gaze_memory.hold_threshold = 1.2  # seconds

        status["wait_for_gaze"] = True
        wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
        status["wait_for_gaze"] = False
        worker.gaze_memory.hold_threshold = original_threshold

        # Place tool after confirmed gaze check
        status["current_slide"] = place_slide
        robot.movel(pose=pose_tool_square_place, a=acc, v=vel)
        gripper.move_and_wait_for_pos(0, 255, 255)
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
        time.sleep(2.0)

        # Wait for gaze?
        time.sleep(10)  # ensures tool is picked up by human before next iteration
        status["wait_for_gaze"] = True
        wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
        status["wait_for_gaze"] = False

        #Check if tool is placed back
        status["wait_for_detection"] = True
        status["detection_result"] = None

        while status["detection_result"] is None:
            time.sleep(0.05)

        detected_pose = status["detection_result"]
        status["wait_for_detection"] = False

        # Check detection success
        if detected_pose is None:
            status["current_slide"] = SLIDE_THANK_YOU  # fallback or error slide


    # Abrupt change to test trust.
    time.sleep(4.0)
    gripper.move_and_wait_for_pos(255, 255, 255)
    time.sleep(0.5)
    gripper.move_and_wait_for_pos(0, 255, 255)
    worker.gaze_memory.hold_threshold = 1.0

    for idx, point in enumerate(workspace_points):

        status["wait_for_gaze"] = True

        if wait_for_gaze_command(worker, GAZE_COMMAND_ZONE):

            status["wait_for_gaze"] = False
            slow_vel = vel * 0.1  # slow if gaze detected
            print(f"[Trust Test] Gaze detected! Moving slowly to waypoint {idx + 1}")
        else:
            status["wait_for_gaze"] = False
            slow_vel = vel
            print(f"[Trust Test] Moving normally to waypoint {idx + 1}")

        robot.movel(pose=point, a=acc, v=slow_vel)
        time.sleep(1.0)  # pause slightly at each waypoint

    worker.gaze_memory.hold_threshold = original_threshold
    status["current_slide"] = SLIDE_RETURN_HOME
    time.sleep(2.0)

    # Connect battery
    status["current_slide"] = SLIDE_BATTERY
    print(f"[Step {step_counter}] Returning home")
    status["wait_for_gaze"] = True
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    status["wait_for_gaze"] = False

    robot.movel(pose=home_joints, a=acc, v=vel)

    # Now show finish slide AND WAIT
    time.sleep(3.0)
    status["current_slide"] = len(instruction_slides) - 1


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
        worker = GazeWorker(cam_id=0, record_filename="recordings/gaze_video_1.mp4")
        worker.gaze_memory = GazeMemory(hold_threshold=0.1, memory_window=1.0)
        worker.start()
        # worker.start_recording("gaze_window_recording.mp4")
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

            """
            #  Check detection request
            if status.get("wait_for_detection"):
                print("[Main] Running object detection...")
                detected_pose = blue_detector.detect()  # blocking call
                status["detection_result"] = detected_pose
                status["wait_for_detection"] = False
                print("[Main] Detection done:", detected_pose)
                continue
            """
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
        # worker.stop_recording()
        stop_worker_gracefully(worker)
        cv2.destroyAllWindows()
        print(" Program successfully shut down.")

