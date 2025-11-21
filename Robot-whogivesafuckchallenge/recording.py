import cv2
from camera import CameraManager
import time

def main():
    # Initialize CameraManager (choose camera IDs to try)
    camera_ids = [1]  # try multiple cameras if available
    cam_manager = CameraManager(camera_ids=camera_ids)
    active_cam = camera_ids[0]

    # VideoWriter setup
    filename = f"camera_{active_cam}_{int(time.time())}.avi"
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))

    print(f"Recording from camera {active_cam} to file {filename}")
    recording = True

    try:
        while True:
            success, frame = cam_manager.get_frame(active_cam)
            if not success or frame is None:
                print("No frame received")
                continue

            # Show the frame
            cv2.imshow(f"Camera {active_cam} Live", frame)

            # Write frame to file if recording
            if recording:
                out.write(frame)

            # Key controls
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('r'):
                recording = not recording
                print("Recording toggled:", recording)

    finally:
        out.release()
        cam_manager.stop_all()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
