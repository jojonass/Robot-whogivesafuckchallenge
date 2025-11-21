# object_detection_trigger.py

import cv2
import numpy as np
import time

LOWER_ORANGE = (5, 100, 100)
UPPER_ORANGE = (25, 255, 255)
MIN_AREA = 1500
_KERNEL = np.ones((5, 5), np.uint8)


class ObjectDetectionTrigger:
    """
    Detects a stable orange rectangle and computes a TCP pose D
    based on camera pixel coordinates and calibrated mm-per-pixel scales.
    """

    def __init__(
        self,
        cam_index=0,
        width=None,
        height=None,
        stability_time=1.5,
        calib_file="camera_scales.npz",
        base_tcp_pose=None,
        timeout=5.0,
    ):
        """
        cam_index:       OpenCV camera index.
        width, height:   Optional; if None, use calibration image size.
        stability_time:  How long (s) the object must stay stable.
        calib_file:      .npz with scale_x_mm_per_px, scale_y_mm_per_px, image_width, image_height.
        base_tcp_pose:   Reference TCP pose [x, y, z, rx, ry, rz] in meters.
        timeout:         Max time (s) for trying to get a stable detection.
        """
        self.stability_time = stability_time
        self.calib_file = calib_file
        self.base_tcp_pose = base_tcp_pose  # [x, y, z, rx, ry, rz] in meters
        self.detection_triggered = False
        self.target_pose = None
        self.last_center = None
        self.last_seen_time = None

        self._load_calibration()

        if width is None:
            width = self.image_width
        if height is None:
            height = self.image_height

        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {cam_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self._run_detection_loop(timeout=timeout)

        if self.detection_triggered and self.base_tcp_pose is not None and self.last_center is not None:
            self._compute_target_pose_from_center(self.last_center)
        else:
            if self.detection_triggered and self.base_tcp_pose is None:
                print("[ObjectDetectionTrigger] Detection succeeded but base_tcp_pose is None; "
                      "cannot compute target pose D.")

    def __bool__(self):
        """Allows 'if detector:' syntax."""
        return bool(self.detection_triggered)

    def get_target_pose(self):
        """Return target TCP pose D [x, y, z, rx, ry, rz] in meters, or None."""
        return self.target_pose

    # --- Calibration loading ---

    def _load_calibration(self):
        try:
            data = np.load(self.calib_file)
            self.image_width = int(data["image_width"])
            self.image_height = int(data["image_height"])
            self.scale_x_mm_per_px = float(data["scale_x_mm_per_px"])
            self.scale_y_mm_per_px = float(data["scale_y_mm_per_px"])

            self.scale_x_m_per_px = self.scale_x_mm_per_px / 1000.0
            self.scale_y_m_per_px = self.scale_y_mm_per_px / 1000.0

            self.cx = self.image_width / 2.0
            self.cy = self.image_height / 2.0

            print(f"[ObjectDetectionTrigger] Loaded calibration from {self.calib_file}")
            print(f"  image_width  = {self.image_width}")
            print(f"  image_height = {self.image_height}")
            print(f"  scale_x_mm_per_px = {self.scale_x_mm_per_px:.6f}")
            print(f"  scale_y_mm_per_px = {self.scale_y_mm_per_px:.6f}")

        except Exception as e:
            raise RuntimeError(f"Failed to load calibration file '{self.calib_file}': {e}")

    # --- Orange rectangle detection ---

    def _detect_orange(self, frame):
        """Returns (cx, cy) if an orange rectangle is found, else None."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                if area > best_area:
                    best_area = area
                    best = cnt

        if best is None:
            return None

        rect = cv2.minAreaRect(best)
        (cx, cy), _, _ = rect
        return int(cx), int(cy)

    # --- Detection loop ---

    def _run_detection_loop(self, timeout=5.0):
        """Run the detection loop until a stable orange object is detected or timeout."""
        print("[ObjectDetectionTrigger] Starting detection loop. Waiting for stable object...")

        start_time = time.time()

        while True:
            if timeout is not None and (time.time() - start_time > timeout):
                print("[ObjectDetectionTrigger] Timeout reached without stable detection.")
                break

            ret, frame = self.cap.read()
            if not ret:
                continue

            detection = self._detect_orange(frame)
            now = time.time()

            if detection:
                cx, cy = detection

                if self.last_center is None:
                    self.last_center = (cx, cy)
                    self.last_seen_time = now
                else:
                    dx = abs(cx - self.last_center[0])
                    dy = abs(cy - self.last_center[1])
                    dist = (dx ** 2 + dy ** 2) ** 0.5

                    if dist < 10:  # within 10 pixels = stable
                        if now - self.last_seen_time > self.stability_time:
                            print("[ObjectDetectionTrigger] Object held steady, detection triggered.")
                            self.detection_triggered = True
                            self.last_center = (cx, cy)
                            break
                    else:
                        self.last_center = (cx, cy)
                        self.last_seen_time = now
            else:
                self.last_center = None
                self.last_seen_time = None

        self.cap.release()

    # --- Pixel → robot pose (point D) ---

    def _compute_target_pose_from_center(self, center_uv):
        """
        Given pixel center (u, v) and a base_tcp_pose, compute the target TCP pose D in meters.
        Z is kept equal to base_tcp_pose z.
        """
        if self.base_tcp_pose is None:
            return

        u, v = center_uv

        delta_u = u - self.cx
        delta_v = v - self.cy

        delta_x = delta_u * self.scale_x_m_per_px
        delta_y = delta_v * self.scale_y_m_per_px

        bx, by, bz, brx, bry, brz = self.base_tcp_pose

        tx = bx + delta_x
        ty = by + delta_y
        tz = bz  # fixed Z

        self.target_pose = [tx, ty, tz, brx, bry, brz]
        print(f"[ObjectDetectionTrigger] Computed target pose D: {self.target_pose}")
