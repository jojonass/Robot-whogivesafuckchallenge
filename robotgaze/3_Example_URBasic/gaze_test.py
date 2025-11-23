import time
import cv2
from camera import CameraManager
from eye_contact_detector import EyeContactDetector
from New_instructions import SlideViewer, instruction_slides

import threading

# -------------------------------
# Gaze Worker
# -------------------------------
class GazeWorker(threading.Thread):
    def __init__(self, camera_manager, cam_id=0):
        super().__init__()
        self.camera_manager = camera_manager
        self.cam_id = cam_id
        self.detector = EyeContactDetector(camera_manager)
        self.running = True
        self.is_calibrated = False
        self.gaze_zone = None  # initialize

    def run(self):
        print("[GazeWorker] Starting...")
        self.detector.start_calibration(self.cam_id)
        while self.running:
            ret, frame = self.camera_manager.get_frame(self.cam_id)
            if not ret:
                continue

            gaze_data = self.detector.detect_and_draw(self.cam_id, frame, is_active=True)

            # Update gaze_zone once calibration is done
            if gaze_data['zone_label'] != "Calibrating":
                self.is_calibrated = True
                self.gaze_zone = gaze_data['zone_label']
                print(f"[GazeWorker] Gaze detected: {self.gaze_zone}")

            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        cv2.destroyWindow("Camera Feed")
        print("[GazeWorker] Stopped.")



# -------------------------------
# Main Robot Routine
# -------------------------------
def main():
    cam_manager = CameraManager(camera_ids=[0])
    slide_viewer = SlideViewer(scale=0.8)

    # ---------------------------
    # Step 1: Show "Standby / Calibration" slide
    # ---------------------------
    slide_viewer.show_slide_blocking_robot(instruction_slides[0])

    # ---------------------------
    # Step 2: Start gaze worker
    # ---------------------------
    gaze_worker = GazeWorker(cam_manager)
    gaze_worker.start()

    # ---------------------------
    # Step 3: Wait for calibration
    # ---------------------------
    print("Waiting for gaze calibration...")
    while not gaze_worker.is_calibrated:
        time.sleep(0.1)
    print("Calibration complete!")

    # ---------------------------
    # Step 4: Show remaining slides with gaze/button control
    # ---------------------------
    try:
        for slide in instruction_slides[1:]:
            img = slide_viewer.prepare_slide_image(slide)
            cv2.imshow(slide_viewer.window_name, img)

            # If it's the "Ready" slide, wait for user button click (simulated with 'y')
            if slide.get("text_only", False) and "Ready" in slide["title"]:
                ready = False
                while not ready:
                    key = cv2.waitKey(50)
                    if key == ord('y'):
                        ready = True
                    time.sleep(0.05)

            # For other slides, optionally wait for gaze "Right" to advance
            elif slide.get("text_only", False) is False:
                target_zone = "Right"
                while gaze_worker.gaze_zone != target_zone:
                    time.sleep(0.05)

            time.sleep(1)  # Small delay between slides

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        # ---------------------------
        # Cleanup
        # ---------------------------
        gaze_worker.running = False
        gaze_worker.join()
        cam_manager.stop_all()
        cv2.destroyAllWindows()
        print("Routine finished and cleaned up.")


if __name__ == "__main__":
    main()
