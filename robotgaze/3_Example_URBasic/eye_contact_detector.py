import numpy as np
import cv2
import mediapipe as mp
import time
from collections import deque
# The import below assumes the CameraManager class exists in camera.py
# from camera import CameraManager


AUTO_HIDE_AFTER_CALIB = True

class EyeContactDetector:
    """
    Multi-camera eye contact and gaze zone detector.
    Each camera maintains its own calibration, gaze history, and zones.
    """

    LEFT_IRIS = [474, 475, 476, 477]
    RIGHT_IRIS = [469, 470, 471, 472]
    SMOOTHING_WINDOW = 5  # Number of recent values to average for smooth gaze tracking
    CALIBRATION_TIME = 8.0  # Seconds for calibration

    # NEW LOGIC: Use 0.7 as the fixed split point for Left/Right zones
    GAZE_SPLIT_THRESHOLD = 0.6

    def __init__(self, camera_manager):
        self.camera_manager = camera_manager

        # Mediapipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        # Initialize the Face Mesh model once
        self.face_mesh_model = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        # Per-camera data
        self.calibrated_data = {}  # cam_id -> horizontal center (0.0 to 1.0)
        self.recent_gaze_x = {}  # cam_id -> deque
        self.needs_calibration = {}  # cam_id -> bool
        self.calibration_start_time = {}  # cam_id -> float
        self.calibrating_data = {}  # cam_id -> list

    def start_calibration(self, cam_id):
        """Initializes the state for a new calibration period for a given camera."""
        print(f"[{cam_id}] Initializing calibration state...")
        self.needs_calibration[cam_id] = True
        self.calibration_start_time[cam_id] = time.time()
        self.calibrating_data[cam_id] = []
        # Set a default center until calibration is complete
        self.calibrated_data[cam_id] = 0.5

    def _get_iris_position(self, results, image_width, image_height):
        """Calculates the normalized horizontal iris position (0.0 to 1.0)."""
        if not results.multi_face_landmarks:
            return None

        face_landmarks = results.multi_face_landmarks[0]

        def get_center(landmarks, indices):
            x_sum, y_sum = 0, 0
            for i in indices:
                point = landmarks.landmark[i]
                x_sum += point.x
                y_sum += point.y
            return x_sum / len(indices), y_sum / len(indices)

        # Average the two iris centers for the overall gaze position
        left_center_x, _ = get_center(face_landmarks, self.LEFT_IRIS)
        right_center_x, _ = get_center(face_landmarks, self.RIGHT_IRIS)
        gaze_x = (left_center_x + right_center_x) / 2

        return gaze_x

    def detect_and_draw(self, cam_id, image, is_active=True):
        """
        Performs detection, updates gaze state, and draws feedback on the image.
        Returns gaze data dictionary or None.

        Returns:
          {'is_detected': bool, 'zone_label': str | None, 'normalized_x': float | None}
        """

        # If the worker is not active, skip all the expensive CV/MediaPipe work
        if not is_active:
            return {'is_detected': False, 'zone_label': None, 'normalized_x': None}

        h, w, _ = image.shape
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.face_mesh_model.process(image_rgb)
        image_rgb.flags.writeable = True

        raw_x = self._get_iris_position(results, w, h)

        if raw_x is None:
            # No face detected
            return {'is_detected': False, 'zone_label': None, 'normalized_x': None}

        # --- Gaze Smoothing ---
        if cam_id not in self.recent_gaze_x:
            self.recent_gaze_x[cam_id] = deque([raw_x] * self.SMOOTHING_WINDOW, maxlen=self.SMOOTHING_WINDOW)

        self.recent_gaze_x[cam_id].append(raw_x)
        smooth_x = np.median(list(self.recent_gaze_x[cam_id]))

        # --- CALIBRATION LOGIC ---
        if self.needs_calibration.get(cam_id, False):
            self.calibrating_data[cam_id].append(smooth_x)

            if time.time() - self.calibration_start_time[cam_id] >= self.CALIBRATION_TIME:
                # Calibration time is up, finalize the center
                if self.calibrating_data[cam_id]:
                    self.calibrated_data[cam_id] = np.median(self.calibrating_data[cam_id])
                    # Note: We are using a fixed 0.7 split, so calibration feedback is purely informational.
                self.needs_calibration[cam_id] = False
                self.calibrating_data[cam_id] = []

            # Even if calibrating, we know a face is detected
            return {'is_detected': True, 'zone_label': "Calibrating", 'normalized_x': smooth_x}

        # --- NORMAL OPERATION (after calibration) ---

        # Determine zone based on fixed 0.7 split (Left is < 0.7, Right is >= 0.7)
        if smooth_x >= self.GAZE_SPLIT_THRESHOLD:
            zone_label = "Left"
        else:
            zone_label = "Right"

        # Draw the fixed split line at 0.7
        split_x_px = int(self.GAZE_SPLIT_THRESHOLD * w)
        # Use a slightly darker color for the line to contrast with face mesh points
        cv2.line(image, (split_x_px, 0), (split_x_px, h), (0, 150, 0), 2)

        # Draw gaze marker
        cv2.circle(image, (int(smooth_x * w), h // 2), 10, (0, 0, 255), -1)

        cv2.putText(image, f"Gaze: {zone_label}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return {'is_detected': True, 'zone_label': zone_label, 'normalized_x': smooth_x}

    def detect_eye_contact(self, cam_id):
        """
        Retrieves frame from CameraManager, performs detection, and returns data
        in the format expected by EyeContactApp.update_frame().

        Expected GUI Format: (frame, eye_contact_bool, zone_label, proximity_float)
        """
        # 1. Get the latest frame
        ret, frame = self.camera_manager.get_frame(cam_id)
        if not ret or frame is None:
            # Return empty data if frame read failed
            return None, False, "--", 0.0

        # Determine if this is the active camera (based on GUI logic for drawing)
        # Note: The original code doesn't pass 'is_active', so we assume we always draw.

        # 2. Perform detection and drawing
        gaze_data = self.detect_and_draw(cam_id, frame)

        # 3. Format output for the GUI

        # In this specific setup, "Eye Contact" is defined as a detected face
        # with a valid zone label (not "Calibrating").
        eye_contact = gaze_data['is_detected'] and gaze_data['zone_label'] not in ["--", "Calibrating"]
        zone = gaze_data['zone_label']

        # Use a placeholder for proximity, as it's not calculated here
        proximity = 0.0

        return frame, eye_contact, zone, proximity

        # ----------------- CALIBRATION -----------------
    def calibrate(self):
        """
        [EyeContactApp Method] Initiates calibration for the currently active camera.
        It calls the 'calibrate' method on the EyeContactDetector instance.
        """
        # This calls the now-defined 'calibrate' method in EyeContactDetector
        self.detector.calibrate(self.active_cam)
        self.label.config(text="Calibrating...", fg="orange")
        print(f"Calibration started for camera {self.active_cam}")

        # Schedule GUI hide after calibration
        if AUTO_HIDE_AFTER_CALIB:
            # Access CALIBRATION_TIME directly from the detector class
            calibration_time = self.detector.CALIBRATION_TIME
            self.root.after(int(calibration_time * 1000), self.hide_window)
