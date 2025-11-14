import numpy as np
import cv2
import mediapipe as mp
import time
from collections import deque
from camera import CameraManager


class EyeContactDetector:
    """
    Multi-camera eye contact and gaze zone detector.
    Each camera maintains its own calibration, gaze history, and zones.
    """

    LEFT_IRIS = [474, 475, 476, 477]
    RIGHT_IRIS = [469, 470, 471, 472]

    def __init__(self, camera_manager: CameraManager):
        self.camera_manager = camera_manager

        # Mediapipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

        # Per-camera data
        self.face_mesh_per_cam = {}
        self.calibrated_data = {}        # cam_id -> horizontal center
        self.recent_gaze_x = {}          # cam_id -> deque
        self.needs_calibration = {}      # cam_id -> bool
        self.calibration_start_time = {} # cam_id -> float
        self.calibrating_data = {}       # cam_id -> list

        # Constants
        self.CALIBRATION_TIME = 5.0  # seconds
        self.GAZE_THRESHOLD = 0.02   # small threshold to define center
        self.HIST_LEN = 5             # smoothing history

    def _init_camera_data(self, cam_id):
        if cam_id not in self.face_mesh_per_cam:
            self.face_mesh_per_cam[cam_id] = self.mp_face_mesh.FaceMesh(
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.recent_gaze_x[cam_id] = deque(maxlen=self.HIST_LEN)
            self.needs_calibration[cam_id] = True
            self.calibration_start_time[cam_id] = None
            self.calibrating_data[cam_id] = []
            self.calibrated_data[cam_id] = 0.5  # default center

    def calibrate(self, cam_id):
        self._init_camera_data(cam_id)
        self.needs_calibration[cam_id] = True
        self.calibration_start_time[cam_id] = time.time()
        self.calibrating_data[cam_id] = []

    def _get_iris_center(self, landmarks, iris_indices, image_shape):
        h, w, _ = image_shape
        xs = [landmarks[i].x * w for i in iris_indices]
        return np.mean(xs) / w  # normalized 0..1

    def detect_eye_contact(self, cam_id):
        self._init_camera_data(cam_id)
        start = time.time()
        try:
            success, image = self.camera_manager.get_frame(cam_id)
        except ValueError:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, f"Cam {cam_id} Error", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            return blank, False, "--", "---"

        if not success or image is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, f"Cam {cam_id} Not Found", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            return blank, False, "--", "---"

        image.flags.writeable = False
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.face_mesh_per_cam[cam_id].process(image_rgb)
        image.flags.writeable = True
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        eye_contact = False
        zone_label = "--"
        percentage_text = "--"

        if results.multi_face_landmarks:
            lm = results.multi_face_landmarks[0].landmark

            # Average iris horizontal position
            left_iris_x = self._get_iris_center(lm, self.LEFT_IRIS, image.shape)
            right_iris_x = self._get_iris_center(lm, self.RIGHT_IRIS, image.shape)
            gaze_x = (left_iris_x + right_iris_x) / 2

            # Calibration
            if self.needs_calibration[cam_id]:
                self.calibrating_data[cam_id].append(gaze_x)
                elapsed = time.time() - self.calibration_start_time[cam_id]
                cv2.putText(image, f"CALIBRATING ({int(self.CALIBRATION_TIME - elapsed)+1})",
                            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                if elapsed >= self.CALIBRATION_TIME:
                    self.calibrated_data[cam_id] = np.mean(self.calibrating_data[cam_id])
                    self.needs_calibration[cam_id] = False
                    print(f"Camera {cam_id} calibrated: center={self.calibrated_data[cam_id]:.2f}")
                return image, False, "Calibrating", "---"

            # Smooth gaze
            self.recent_gaze_x[cam_id].append(gaze_x)
            smooth_x = np.mean(self.recent_gaze_x[cam_id])

            # Determine Left/Right zones (switched)
            diff = smooth_x - self.calibrated_data[cam_id]
            if diff < 0:
                zone_label = "Right"  # switched
                percentage = min(100, int(abs(diff)/self.calibrated_data[cam_id]*100))
            else:
                zone_label = "Left"   # switched
                percentage = min(100, int(abs(diff)/(1-self.calibrated_data[cam_id])*100))
            percentage_text = f"{percentage}%"

            # Eye contact if near center
            eye_contact = abs(diff) < self.GAZE_THRESHOLD

            # Draw vertical calibration line
            h, w, _ = image.shape
            center_x_px = int(self.calibrated_data[cam_id]*w)
            cv2.line(image, (center_x_px, 0), (center_x_px, h), (0,255,255), 2)

            # Draw gaze marker
            cv2.circle(image, (int(smooth_x*w), h//2), 10, (0,255,0), -1)

            # Horizontal bar
            bar_y = int(h*0.9)
            cv2.rectangle(image, (50, bar_y-10), (w-50, bar_y+10), (200,200,200), 2)
            marker_x = int(50 + (w-100)*smooth_x)
            cv2.circle(image, (marker_x, bar_y), 8, (0,0,255), -1)

            # Display zone and percentage
            cv2.putText(image, f"Zone: {zone_label} ({percentage_text})", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

        fps = int(1 / (time.time() - start + 1e-6))
        cv2.putText(image, f"FPS: {fps}", (20, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        return image, eye_contact, zone_label, percentage_text

    def release(self):
        for mesh in self.face_mesh_per_cam.values():
            mesh.close()
        cv2.destroyAllWindows()
