import cv2
import time
import URBasic
from New_instructions import SlideViewer, instruction_slides
from gaze_manager import GazeWorker, GazeMemory, wait_for_gaze_command
from camera import CameraManager

GAZE_COMMAND_ZONE = 'Right'

# ------------------------
# Camera manager
# ------------------------
cam_manager = CameraManager(camera_ids=[0])

# ------------------------
# Robot configuration
# ------------------------
host = "192.168.0.9"
acc = 0.4
vel = 0.5

# Joint positions
pose_central_tool = [0.244, -0.32, 0.3, -3.122, -0.03, -0.123]
pose_A_approach = pose_central_tool
pose_A_grasp = [0.25, -0.315, 0.152, -3.122, -0.03, -0.123]
pose_tool_square_approach = [-0.028, -0.371, 0.282, -3.122, -0.03, -0.12]
pose_tool_square_place = [-0.041, -0.393, 0.155, -3.122, -0.03, -0.12]
home_joints = [0.06, -0.322, 0.405, -3.122, -0.03, -0.123]

# ------------------------
# Camera calibration
# ------------------------
def run_camera_calibration(camera_id=0, seconds=7, hide_after=True):
    import cv2, time
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print("❌ ERROR: Camera could not be opened")
        return False

    start_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret: break

        elapsed = time.time() - start_time
        remaining = int(seconds - elapsed)
        text = f"Calibrating: {remaining}" if remaining >= 0 else "Calibration Complete!"
        color = (255,255,255) if remaining >= 0 else (0,255,0)
        font_scale = 2 if remaining >= 0 else 1.5
        thickness = 4 if remaining >= 0 else 3
        cv2.putText(frame, text, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

        cv2.imshow("Camera Calibration", frame)
        if elapsed > seconds + 1: break
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    if hide_after:
        cap.release()
        cv2.destroyAllWindows()
        print("🟦 Camera calibration done, window closed.")
        return True

    return True

# ------------------------
# Main
# ------------------------
if __name__ == "__main__":
    # Initialize robot
    robotModel = URBasic.robotModel.RobotModel()
    robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModel)
    robot.reset_error()

    # Initialize SlideViewer
    slide_viewer = SlideViewer(scale=0.8)

    # Step 1: Show initial standby slide
    img = slide_viewer.prepare_slide_image(instruction_slides[0])
    cv2.imshow(slide_viewer.window_name, img)
    cv2.waitKey(1)
    time.sleep(2)

    # Step 2: Run camera calibration
    run_camera_calibration(camera_id=0, seconds=7, hide_after=True)

    # Step 3: Start GazeWorker, skip internal calibration
    worker = GazeWorker(camera_manager=cam_manager, cam_id=0)
    worker.gaze_memory = GazeMemory(hold_threshold=0.5, memory_window=3.0)
    worker.skip_calibration = True  # <- IMPORTANT: skip internal calibration
    worker.is_active = False  # do not show feed
    worker.start()
    print("[Main] GazeWorker started without recalibration.")

    # Step 4: Show slide 1
    img = slide_viewer.prepare_slide_image(instruction_slides[1])
    cv2.imshow(slide_viewer.window_name, img)
    cv2.waitKey(1)
    print("Slide 1 shown, waiting for user gaze...")

    # Example: wait for gaze before proceeding
    wait_for_gaze_command(worker, GAZE_COMMAND_ZONE)
    print("Slide 1 done, robot routine starting...")

    # Step 5: Run your robot sequence here
    # ExampleurScript(worker, slide_viewer)
    # ------------------------
    # Keep slide window open until user closes
    while True:
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    # Stop worker
    worker.running = False
    worker.join()
    cv2.destroyAllWindows()
    print("✅ Done.")
