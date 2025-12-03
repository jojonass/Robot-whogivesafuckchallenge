import cv2
import os
import time
import threading
import URBasic
import robotiq_gripper
from object_detection_trigger import ObjectDetectionTrigger
from Instructions_button import SlideViewer

slide_viewer = SlideViewer(slide_folder="Slides_button")
instruction_slides = slide_viewer.slides  # use the actual slide dictionaries



# -------------------------------
# Shared robot/GUI state
# -------------------------------
status = {
    "current_slide": 0,
    "button_pressed": False,
    "wait_for_detection": False,
    "detection_result": None

}

"""
blue_detector = ObjectDetectionTrigger(
    cam_index=1,
    stability_time=2.0,   # wait for 2 seconds of stability
    timeout=9999.0        # effectively wait "forever"
)
"""


host = "192.168.0.9"
#  host '192.168.1.10'
acc = 0.5
vel = 0.5

"""
detector = ObjectDetectionTrigger(
    cam_index=1,
    stability_time=1.5,
    timeout=9999.0   # wait effectively "forever"
)
"""
# -------------------------------
# SlideViewer class
# -------------------------------
# -------------------------------
# SPACEBAR wait function
# -------------------------------
def wait_for_button(status):
    """Waits until main thread sets status['button_pressed'] = True."""
    print("➡️ Waiting for SPACEBAR…")
    while not status.get("button_pressed", False):
        time.sleep(0.05)
    status["button_pressed"] = False
    print(" SPACEBAR pressed — continuing\n")

# -------------------------------
# Robot routine
# -------------------------------
def ExampleurScript(status):
    step = 0

    # Example poses
    pose_central_tool = [-0.249, 0.348, 0.355, -3.148, 0.028, 0.022]
    pose_A_approach = pose_central_tool
    pose_A_grasp = [-0.257, 0.433, 0.136, -3.148, 0.028, 0.022]
    pose_tool_square_approach = [-0.336, 0.196, 0.377, -3.148, 0.028, 0.022]
    pose_tool_square_place = [-0.434, 0.263, 0.136, -3.148, 0.028, 0.022]
    home_joints = [-0.380, -0.149, 0.3, -3.148, 0.028, 0.022]
    workspace_points = [
        [-0.25, 0.25, 0.35, -2.364, -2.26, 0.171],
        [-0.22, 0.22, 0.30, -2.364, -2.26, 0.171],
        [-0.20, 0.20, 0.28, -2.364, -2.26, 0.171],
    ]

    robotModel = URBasic.robotModel.RobotModel()
    robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModel)
    robot.reset_error()

    gripper = robotiq_gripper.RobotiqGripper()
    gripper.connect(host, 63352)
    gripper.activate()

    # ---------------------------
    # Slide 1
    # ---------------------------
    step += 1
    slide = 1
    status["current_slide"] = slide
    print(f"[Step {step}] Showing Slide 1")
    wait_for_button(status)

    tools = [{"name": "Tool M4", "approach": pose_A_approach, "grasp": pose_A_grasp}]

    for i, tool in enumerate(tools, start=1):


        # Step 2: move to central tool
        print(f"\n=== Iteration {i}: {tool['name']} ===")
        robot.movel(pose=pose_central_tool, a=acc, v=vel)
        step += 1

        # Step 3: Picking up part
        print("[Slide] Pick up part")
        robot.movel(pose=tool["approach"], a=acc, v=vel)
        print("[Step] Grasping")
        robot.movel(pose=tool["grasp"], a=acc, v=vel)
        gripper.move_and_wait_for_pos(255, 255, 255)
        print("[Step] Retreating")
        robot.movel(pose=tool["approach"], a=acc, v=vel)


        # Step 4: dropping off part and returning
        step += 1
        slide += 1
        status["current_slide"] = slide
        wait_for_button(status)
        print("[Step] Moving to tool square approach")
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
        print("[Step] Placing tool")
        robot.movel(pose=pose_tool_square_place, a=acc, v=vel)
        gripper.move_and_wait_for_pos(0, 255, 255)
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)

        # Waiting for tool detection
        # NEW STEP 8: Object Detection

        '''
        # --- STEP 5: object Detection → Pick-Up ---
        step_counter += 1
        print(f"[Step {step_counter}] Moving back to approach tool for verification.")
        print("[Robot] Requesting object detection...")
        status["wait_for_detection"] = True
        status["detection_result"] = None
        

        # Wait until detection thread updates the result
        while status["detection_result"] is None:
            time.sleep(0.05)
        detected_pose = status["detection_result"]
        status["wait_for_detection"] = False
        print("[Robot] Detection returned:", detected_pose)
        # Check detection success
        if detected_pose is None:
            print("[Robot] ERROR: Detection failed. Stopping routine.")
        status["current_slide"] = 0  # fallback or error slide
        return
        
        print("[Robot] Object detection successful.")

        #  Picking up the object
        step_counter += 1
        print(f"[Step {step_counter}] Picking up object using detected pose.")
        robot.movel(pose=pose_tool_square_place, a=acc, v=vel)
        gripper.move_and_wait_for_pos(255, 255, 255)  # (optional) close gripper

        # Retreat
        robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
        robot.movel(pose=pose_A_approach, a=acc, v=vel)
        robot.movel(pose=pose_A_grasp, a=acc, v=vel)
        gripper.move_and_wait_for_pos(0, 255, 255)

        '''

        # step 9:  Abrupt change to test trust.

        slide += 1
        status["current_slide"] = slide
        time.sleep(2.0)

        # Small open/close gesture
        status["current_slide"] = len(instruction_slides) - 5
        time.sleep(4.0)
        gripper.move_and_wait_for_pos(255, 255, 255)
        time.sleep(0.5)
        gripper.move_and_wait_for_pos(0, 255, 255)


        print("=== TRUST TEST STARTED ===")
        print("Press SPACEBAR during the motion to simulate 'gaze detected'.")
        print("If you don't press it, robot moves normally.")

        for idx, point in enumerate(workspace_points):
            # Always move at low speed
            slow_vel = vel * 0.1
            print(f"[Trust Test] Moving SLOW to waypoint {idx + 1}")

            # Move the robot
            robot.movel(pose=point, a=acc, v=slow_vel)
            time.sleep(1.0)

        status["current_slide"] = len(instruction_slides) - 3
        time.sleep(3.0)

        # Step 10: Return home wait for gaze to confirm finishing

        # Step 10: Return home
        print(f"[Step {step}] Returning home")
        status["current_slide"] = len(instruction_slides) - 2
        wait_for_button(status)

        robot.movel(pose=home_joints, a=acc, v=vel)

        # Final slide — stay here!
        status["current_slide"] = len(instruction_slides) - 1
        print("[Step] Final slide shown. Routine complete!")
        return  # stop thread cleanly


# -------------------------------
# Main program
# -------------------------------
if __name__ == "__main__":
    try:
        # Initialize SlideViewer
        slide_viewer = SlideViewer(slide_folder="Slides_button", scale=0.97)

        # Show first slide
        img = slide_viewer.prepare_slide_image(0)
        cv2.imshow(slide_viewer.window_name, img)
        cv2.waitKey(1)
        time.sleep(1)

        # Start robot thread
        robot_thread = threading.Thread(target=ExampleurScript, args=(status,))
        robot_thread.start()

        current_slide_index = 0

        # GUI loop
        while robot_thread.is_alive():

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

            if slide_index != current_slide_index:
                img = slide_viewer.prepare_slide_image(slide_index)
                cv2.imshow(slide_viewer.window_name, img)
                current_slide_index = slide_index

            key = cv2.waitKey(50) & 0xFF
            if key == ord(" "):
                status["button_pressed"] = True
            elif key == ord("q"):
                break

            time.sleep(0.05)

        robot_thread.join()

        # Keep final slide open until user closes with 'q'
        while cv2.waitKey(100) & 0xFF != ord("q"):
            time.sleep(0.1)

    finally:
        cv2.destroyAllWindows()
        print("Program successfully shut down.")
