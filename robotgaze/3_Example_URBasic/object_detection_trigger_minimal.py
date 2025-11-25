# object_detection_trigger_minimal.py

import cv2
import numpy as np
import time

LOWER_BLUE = (110, 150, 30)
UPPER_BLUE = (130, 255, 100)
MIN_AREA = 1500
_KERNEL = np.ones((5, 5), np.uint8)


class ObjectDetectionTrigger:
    """
    Detects a stable blue rectangle.
    After the object is held still for 'stability_time', detection_triggered becomes True.
    """

    def __init__(self, cam_index=1, stability_time=1.5, timeout=5.0):
        self.stability_time = stability_time
        self.timeout = timeout

        self.detection_triggered = False
        self.last_center = None
        self.last_seen_time = None

        # Open camera
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {cam_index}")

        self._run_detection_loop()

    def __bool__(self):
        return self.detection_triggered

    # --- Blue rectangle detection ---
    def _detect_blue(self, frame):
        """Returns (cx, cy) if a rectangle is found, else None."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
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
    def _run_detection_loop(self):
        print("[Trigger] Starting detection loop...")

        start = time.time()
        while True:
            if time.time() - start > self.timeout:
                print("[Trigger] Timeout, no stable detection.")
                break

            ret, frame = self.cap.read()
            if not ret:
                continue

            detection = self._detect_blue(frame)
            now = time.time()

            if detection:
                cx, cy = detection

                if self.last_center is None:
                    self.last_center = (cx, cy)
                    self.last_seen_time = now
                else:
                    dx = abs(cx - self.last_center[0])
                    dy = abs(cy - self.last_center[1])
                    dist = (dx**2 + dy**2) ** 0.5

                    if dist < 10:  # stable
                        if now - self.last_seen_time >= self.stability_time:
                            print("[Trigger] Object stable -> DETECTED")
                            self.detection_triggered = True
                            break
                    else:
                        self.last_center = (cx, cy)
                        self.last_seen_time = now
            else:
                self.last_center = None
                self.last_seen_time = None

        self.cap.release()
