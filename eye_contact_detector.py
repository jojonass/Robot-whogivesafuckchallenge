import numpy as np
import cv2
import mediapipe as mp
import time
from collections import deque
import threading


class VideoStream:
    """
    A class to handle video capture in a separate thread for increased FPS performance.
    """

    def __init__(self, src=0):
        # Initialize video capture (typically webcam 0)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            print("Error: Could not open video stream.")
            self.stopped = True
            return

        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.lock = threading.Lock()

        # Start the thread to continuously read frames
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        """
        Reads the next frame from the camera in a loop.
        """
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame
            # If capture fails, stop the loop gracefully
            else:
                self.stopped = True

    def read(self):
        """
        Returns the latest frame read by the thread.
        """
        with self.lock:
            return self.ret, self.frame

    def stop(self):
        """
        Stops the thread and releases the video capture object.
        """
        self.stopped = True
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        if hasattr(self, 'cap'):
            self.cap.release()


class EyeContactDetector:
    """
    Main class for real-time eye contact and gaze zone detection using Mediapipe and OpenCV.
    """

    def __init__(self):
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.define_mesh()
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
        self.cap = VideoStream(0)

        # Calibration baseline (Head Pose)
        self.calibrated_x, self.calibrated_y, self.calibrated_z = 6, 0, 0
        # Calibration baseline (Proximity/Distance) - Initialized at 0.0
        self.calibrated_distance = 0.0
        # REDUCED TO 0.05 (5%) for finer distance differentiation
        self.DISTANCE_THRESHOLD_FACTOR = 0.05

        self.blinking = False
        self.EAR_THRESHOLD = 0.18  # Eye Aspect Ratio for blink detection
        self.eye_contact = False
        self._needs_calibration = True  # Set to True to force initial calibration

        # === NEW TIMED CALIBRATION VARIABLES ===
        self.is_calibrating = False
        self.calibration_duration = 5  # 5 seconds for calibration
        self.calibration_start_time = 0
        self.calibration_distances = []
        self.calibration_angles_x = []
        self.calibration_angles_y = []
        self.calibration_angles_z = []

        # === Zone tracking (Gaze-based) ===
        self.NUM_ROWS, self.NUM_COLS = 2, 2
        self.STABLE_FRAMES = 5  # Frames required for a zone to be considered stable
        self.zone_history = deque(maxlen=self.STABLE_FRAMES)
        self.stable_zone = None

        # Labels for the 5 gaze zones
        self.ZONE_LABELS = {
            1: 'tl', 2: 'tr', 3: 'bl',
            4: 'br', 5: 'center'
        }

        # Gaze sensitivity thresholds (how far eyes must look to be outside the 'center' zone)
        self.GAZE_THRESH_X = 0.05
        self.GAZE_THRESH_Y = 0.05

    def define_mesh(self):
        """Initializes the Mediapipe FaceMesh object."""
        return self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5
        )

    def calculate_gaze_direction(self, iris_landmarks, eye_landmarks):
        """Calculates a normalized vector representing eye gaze direction."""
        iris_coords = np.array([(lm.x, lm.y, lm.z) for lm in iris_landmarks])
        eye_coords = np.array([(lm.x, lm.y, lm.z) for lm in eye_landmarks])
        iris_center = np.mean(iris_coords, axis=0)
        eye_center = np.mean(eye_coords, axis=0)
        gaze_vector = iris_center - eye_center
        return gaze_vector / np.linalg.norm(gaze_vector)

    def calculate_EAR(self, eye_landmarks):
        """Calculates the Eye Aspect Ratio (EAR) for blink detection."""

        def landmark_to_point(landmark):
            return np.array([landmark.x, landmark.y])

        # Vertical distances
        A = np.linalg.norm(landmark_to_point(eye_landmarks[1]) - landmark_to_point(eye_landmarks[5]))
        B = np.linalg.norm(landmark_to_point(eye_landmarks[2]) - landmark_to_point(eye_landmarks[4]))
        # Horizontal distance
        C = np.linalg.norm(landmark_to_point(eye_landmarks[0]) - landmark_to_point(eye_landmarks[3]))
        # EAR formula
        return (A + B) / (2.0 * C)

    def get_face_proximity(self, face_landmarks):
        """
        Calculates a distance metric based on the physical pixel distance between the eyes.
        This serves as a proxy for face-to-camera distance (Z-depth).
        Landmarks used: 33 (Left eye inner corner) and 263 (Right eye inner corner).
        """
        lm_33 = face_landmarks.landmark[33]
        lm_263 = face_landmarks.landmark[263]

        # Use pixel coordinates (x, y) to calculate the distance on the image plane.
        # This distance is inversely proportional to the face's distance from the camera.
        distance = np.linalg.norm(np.array([lm_33.x, lm_33.y]) - np.array([lm_263.x, lm_263.y]))
        return distance

    def classify_proximity(self, current_distance):
        """
        Compares the current face-size-metric to the calibrated value to determine proximity.
        """
        if self.calibrated_distance == 0.0:
            return "Calibrating"

        diff = current_distance - self.calibrated_distance
        threshold = self.calibrated_distance * self.DISTANCE_THRESHOLD_FACTOR

        if diff > threshold:
            # Current distance is significantly larger than calibrated -> face is NEARER
            return "Near"
        elif diff < -threshold:
            # Current distance is significantly smaller than calibrated -> face is FARTHER
            return "Far"
        else:
            return "Normal"

    def draw_gaze_arrow(self, frame, start_point, gaze_direction, length=100, color=(0, 0, 255), thickness=2):
        """Draws a visual indicator (arrow) of the gaze direction from the nose tip."""
        gaze_direction_2d = gaze_direction[:2] * length
        end_point = tuple(map(int, (start_point + gaze_direction_2d)))
        start_point = tuple(map(int, start_point))
        cv2.arrowedLine(frame, start_point, end_point, color, thickness)
        cv2.circle(frame, start_point, 4, (0, 255, 0), -1)  # Nose dot (green)
        cv2.circle(frame, end_point, 4, (255, 0, 0), -1)  # Direction dot (blue)

    def get_gaze_zone(self, gaze_direction):
        """
        Classifies the gaze direction vector into one of 4 corner zones or a center zone.

        Zones: 1: 'tl', 2: 'tr', 3: 'bl', 4: 'br', 5: 'center'
        """
        x, y = gaze_direction[0], gaze_direction[1]

        # 1. Classify Horizontal movement (X)
        if x < -self.GAZE_THRESH_X:
            x_dir = 'left'
        elif x > self.GAZE_THRESH_X:
            x_dir = 'right'
        else:
            x_dir = 'center'

        # 2. Classify Vertical movement (Y)
        # NOTE: Positive Y usually means looking DOWN in MediaPipe's normalized coordinates
        if y < -self.GAZE_THRESH_Y:
            y_dir = 'up'
        elif y > self.GAZE_THRESH_Y:
            y_dir = 'down'
        else:
            y_dir = 'center'

        # 3. Combine X and Y to determine the zone ID
        if x_dir == 'left' and y_dir == 'up':
            return 1  # Top-Left
        elif x_dir == 'right' and y_dir == 'up':
            return 2  # Top-Right
        elif x_dir == 'left' and y_dir == 'down':
            return 3  # Bottom-Left
        elif x_dir == 'right' and y_dir == 'down':
            return 4  # Bottom-Right
        else:
            # Includes 'center'/'center', 'center'/'up', 'left'/'center', etc.
            return 5  # Center (or near-center)

    def get_stable_zone(self, history):
        """
        Returns the most frequent zone if it has been stable (present in >= 80% of history).
        """
        if len(history) < self.STABLE_FRAMES:
            return None
        zones, counts = np.unique(history, return_counts=True)
        max_zone = zones[np.argmax(counts)]
        # Check if the most frequent zone is dominant
        if counts.max() >= self.STABLE_FRAMES * 0.8:
            return max_zone
        return None

    def detect_eye_contact(self):
        """
        The main processing loop to read frame, detect landmarks, calculate poses/gaze,
        and determine eye contact status and zone.

        Returns:
            image (np.array): The video frame with annotations.
            eye_contact (bool): True if looking directly forward.
            zone_label (str or None): The stable Gaze Zone ('TL', 'BR', etc.).
            proximity_text (str): The inferred distance ('Near', 'Normal', 'Far').
        """
        success, image = self.cap.read()
        if not success:
            return image, False, None, "Calibrating"  # Default proximity during no frame

        start = time.time()
        self.eye_contact = False

        # Flip for natural selfie-view and convert color space for Mediapipe
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert back to BGR for OpenCV drawing

        # Default proximity text if no face is detected
        proximity_text = self.classify_proximity(0.0)

        if not results.multi_face_landmarks:
            # If face is lost, but calibration is running, we still need to show the timer
            if self.is_calibrating:
                elapsed_time = time.time() - self.calibration_start_time
                remaining_time = max(0, self.calibration_duration - elapsed_time)
                cv2.putText(image, f"CALIBRATING... {remaining_time:.1f}s", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                            (0, 0, 255), 3)
            return image, False, getattr(self, 'current_zone_label', None), proximity_text

        img_h, img_w, _ = image.shape
        for face_landmarks in results.multi_face_landmarks:

            # --- 1. Head Pose Calculation (needed for both calibration and running) ---
            face_2d, face_3d = [], []
            for idx, lm in enumerate(face_landmarks.landmark):
                if idx in [33, 263, 1, 61, 291, 199]:
                    x_px, y_px = int(lm.x * img_w), int(lm.y * img_h)
                    face_2d.append([x_px, y_px])
                    face_3d.append([x_px, y_px, lm.z])

            face_2d, face_3d = np.array(face_2d, dtype=np.float64), np.array(face_3d, dtype=np.float64)
            focal_length = 1 * img_w
            cam_matrix = np.array([[focal_length, 0, img_h / 2],
                                   [0, focal_length, img_w / 2],
                                   [0, 0, 1]])
            distortion_matrix = np.zeros((4, 1), dtype=np.float64)

            _, rotation_vec, _ = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix, flags=cv2.SOLVEPNP_EPNP)
            rmat, _ = cv2.Rodrigues(rotation_vec)
            angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
            x_raw, y_raw, z_raw = [a * 360 for a in angles]

            # --- 2. Proximity Calculation (needed for both calibration and running) ---
            current_distance = self.get_face_proximity(face_landmarks)

            # --- 3. TIMED CALIBRATION LOGIC ---
            if self._needs_calibration and not self.is_calibrating:
                # START Calibration Phase
                self.is_calibrating = True
                self.calibration_start_time = time.time()
                self.calibration_distances = []
                self.calibration_angles_x = []
                self.calibration_angles_y = []
                self.calibration_angles_z = []

            if self.is_calibrating:
                # RUNNING Calibration Phase
                elapsed_time = time.time() - self.calibration_start_time
                remaining_time = max(0, self.calibration_duration - elapsed_time)

                self.calibration_distances.append(current_distance)
                self.calibration_angles_x.append(x_raw)
                self.calibration_angles_y.append(y_raw)
                self.calibration_angles_z.append(z_raw)

                if remaining_time <= 0:
                    # FINISH Calibration Phase

                    # 3a. Proximity Calibration (Average the collected distances)
                    if self.calibration_distances:
                        self.calibrated_distance = np.mean(self.calibration_distances)

                    # 3b. Head Pose Calibration (Average the collected angles)
                    if self.calibration_angles_x:
                        self.calibrated_x = np.mean(self.calibration_angles_x)
                        self.calibrated_y = np.mean(self.calibration_angles_y)
                        self.calibrated_z = np.mean(self.calibration_angles_z)

                    self.is_calibrating = False
                    self._needs_calibration = False
                    print(f"Calibration finished. Normal Distance: {self.calibrated_distance:.4f}")

                # Override normal display during calibration
                cv2.putText(image, f"CALIBRATING... {remaining_time:.1f}s", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                            (0, 0, 255), 3)
                return image, False, None, "Calibrating"  # Return "Calibrating" as proximity state

            # --- 4. Normal Operation ---
            x, y, z = x_raw, y_raw, z_raw

            # Normalize angles by subtracting calibration offsets
            x -= self.calibrated_x
            y -= self.calibrated_y
            z -= self.calibrated_z

            # Head Pose Text Classification (Used only for display/feedback)
            if y < -8:
                head_text = "Left"
            elif y > 8:
                head_text = "Right"
            elif x < -15:
                head_text = "Down"
            elif x > 8:
                head_text = "Up"
            else:
                head_text = "Forward"

            # --- 5. Gaze Direction and Blink Detection ---
            complete_left_eye = [face_landmarks.landmark[i] for i in [33, 160, 158, 133, 153, 144]]
            complete_right_eye = [face_landmarks.landmark[i] for i in [362, 385, 387, 263, 373, 380]]
            left_ear = self.calculate_EAR(complete_left_eye)
            right_ear = self.calculate_EAR(complete_right_eye)
            avg_ear = (left_ear + right_ear) / 2
            self.blinking = avg_ear < self.EAR_THRESHOLD

            left_iris = [face_landmarks.landmark[468]]
            right_iris = [face_landmarks.landmark[473]]
            left_eye = [face_landmarks.landmark[i] for i in [33, 173]]
            right_eye = [face_landmarks.landmark[i] for i in [398, 263]]
            left_gaze = self.calculate_gaze_direction(left_iris, left_eye)
            right_gaze = self.calculate_gaze_direction(right_iris, right_eye)

            # Gaze Offset is CRUCIAL to correct for average camera alignment
            gaze_offset = [0, 0.31, 0]
            gaze_direction = (left_gaze + right_gaze) / 2 + gaze_offset

            # Gaze Text Classification (Needed only internally for eye_contact check)
            gaze_text = "Forward"
            if gaze_direction[0] < -self.GAZE_THRESH_X:
                gaze_text = "Left"
            elif gaze_direction[0] > self.GAZE_THRESH_X:
                gaze_text = "Right"
            elif gaze_direction[1] < -self.GAZE_THRESH_Y:
                gaze_text = "Up"
            elif gaze_direction[1] > self.GAZE_THRESH_Y:
                gaze_text = "Down"

            # --- 6. Determine Final Eye Contact Status ---
            self.eye_contact = not self.blinking and gaze_text == "Forward"

            # Draw gaze vector arrow
            nose_tip = face_landmarks.landmark[4]
            start_point = np.array([nose_tip.x * img_w, nose_tip.y * img_h])
            self.draw_gaze_arrow(image, start_point, gaze_direction)

            # --- 7. Gaze Zone Detection (Gaze Vector based) ---
            current_zone = self.get_gaze_zone(gaze_direction)
            self.zone_history.append(current_zone)
            new_stable = self.get_stable_zone(self.zone_history)

            if new_stable is not None:
                self.stable_zone = new_stable
                zone_label = self.ZONE_LABELS[self.stable_zone]
                self.current_zone_label = zone_label
            else:
                zone_label = getattr(self, 'current_zone_label', None)

            # Draw grid for visual reference
            for i in range(1, self.NUM_ROWS):
                cv2.line(image, (0, int(img_h * i / self.NUM_ROWS)), (img_w, int(img_h * i / self.NUM_ROWS)),
                         (100, 100, 100), 1)
            for j in range(1, self.NUM_COLS):
                cv2.line(image, (int(img_w * j / self.NUM_COLS), 0), (int(img_w * j / self.NUM_COLS), img_h),
                         (100, 100, 100), 1)

            # --- 8. Display Feedback (Head, Gaze Zone, and new Proximity) ---

            # Proximity Text Classification
            proximity_text = self.classify_proximity(current_distance)

            # Display Head Pose
            cv2.putText(image, f"Head: {head_text}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

            # Display Gaze Zone
            if zone_label:
                zone_display_text = f"Gaze Zone: {zone_label.upper()}"
                (text_w, text_h), _ = cv2.getTextSize(zone_display_text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2)
                zone_y = 70
                cv2.rectangle(image, (15, zone_y - text_h - 10), (25 + text_w, zone_y + 10), (0, 0, 0), -1)
                cv2.putText(image, zone_display_text, (20, zone_y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)

            # Display Proximity
            proximity_y = 110  # Position it below the Gaze Zone
            cv2.putText(image, f"Proximity: {proximity_text}", (20, proximity_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (100, 255, 100), 2)

        # FPS counter
        fps = int(1 / (time.time() - start + 1e-6))
        cv2.putText(image, f'FPS: {fps}', (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # The function now returns all four necessary outputs
        return image, self.eye_contact, getattr(self, 'current_zone_label', None), proximity_text

    def calibrate(self):
        """Sets the flag to recalibrate on the next processed frame."""
        self._needs_calibration = True

    def release(self):
        """Releases the video stream and destroys all OpenCV windows."""
        self.cap.stop()
        cv2.destroyAllWindows()
