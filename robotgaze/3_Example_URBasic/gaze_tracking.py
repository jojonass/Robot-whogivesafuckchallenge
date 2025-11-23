import cv2
import sys
import time

# Import necessary components from modular files
# NOTE: The imported classes are assumed to be correctly defined in their respective files.
from eye_contact_detector import EyeContactDetector
from camera import CameraManager

# Define the window name
WINDOW_NAME = "Eye Gaze Calibration and Zone Detector"
CAMERA_ID = 0  # Default camera ID


class CalibrationApp:
    """
    Main application to run the live camera feed, calibration,
    and gaze zone detection using OpenCV GUI.
    """

    def __init__(self):
        # 1. Find and Initialize Camera
        available_cams = get_all_camera_ids()
        if not available_cams:
            print("ERROR: No camera found. Exiting.")
            sys.exit(1)

        # Use the first available camera ID
        self.cam_id = available_cams[0]
        print(f"Using camera ID: {self.cam_id}")

        # Initialize the camera manager and detector
        self.camera_manager = CameraManager(camera_ids=[self.cam_id])
        self.detector = EyeContactDetector(self.camera_manager)

        # 2. Start Calibration Automatically
        # This function must be available on the detector instance
        self.detector.start_calibration(self.cam_id)
        print(
            f"Calibration started for camera {self.cam_id}. Look straight ahead for {self.detector.CALIBRATION_TIME} seconds.")

        # 3. GUI setup (OpenCV window)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        self.running = True

    def run(self):
        """Main application loop."""
        try:
            while self.running:
                # 1. Get the latest frame, process it, and draw all markers
                # This call relies on detect_eye_contact returning the frame and other data
                frame, _, zone, _ = self.detector.detect_eye_contact(self.cam_id)

                if frame is not None:
                    # 2. Check calibration status
                    is_calibrating = self.detector.needs_calibration.get(self.cam_id, False)

                    if is_calibrating:
                        # Display countdown during calibration
                        time_remaining = self.detector.get_calibration_time_remaining(self.cam_id)

                        # Add a large, clear countdown in the center
                        countdown_text = f"{time_remaining:.0f}"
                        if time_remaining > 0.5:
                            # Calculate text position for centering
                            text_scale = 4
                            text_thickness = 8
                            text_size = \
                            cv2.getTextSize(countdown_text, cv2.FONT_HERSHEY_SIMPLEX, text_scale, text_thickness)[0]
                            text_x = (frame.shape[1] - text_size[0]) // 2
                            text_y = (frame.shape[0] + text_size[1]) // 2

                            # Draw countdown in a prominent color (Red for urgency/focus)
                            cv2.putText(frame, countdown_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, text_scale,
                                        (0, 0, 255), text_thickness)


                    else:
                        # Calibration finished, display the current zone
                        status_text = f"Status: Calibrated | Zone: {zone}"
                        # Draw status in the top left
                        cv2.putText(frame, status_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        # Instructions for a user in the bottom left
                        cv2.putText(frame, "Press 'R' to Recalibrate or 'Q' to Quit", (10, frame.shape[0] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                    # 3. Display the resulting frame
                    cv2.imshow(WINDOW_NAME, frame)
                else:
                    # Handle case where frame acquisition fails
                    print("Warning: Failed to retrieve frame from camera.")
                    time.sleep(0.1)  # Prevent busy waiting

                # 4. Handle user input
                key = cv2.waitKey(1) & 0xFF

                # Press 'Q' to quit
                if key == ord('q'):
                    self.running = False

                # Press 'R' to recalibrate
                if key == ord('r'):
                    if not is_calibrating:
                        print("User triggered recalibration.")
                        self.detector.start_calibration(self.cam_id)
                    else:
                        print("Calibration is already running.")

        except Exception as e:
            # Catch all exceptions during the main loop to ensure cleanup runs
            print(f"An error occurred in the main loop: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources before exiting."""
        print("Cleaning up resources...")
        self.camera_manager.stop_all()  # Ensure camera thread stops and resources are released
        cv2.destroyAllWindows()
        print("Application closed.")


if __name__ == "__main__":
    # Ensure this script is the entry point to run the application
    app = CalibrationApp()
    app.run()