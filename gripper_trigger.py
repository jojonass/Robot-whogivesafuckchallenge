import cv2
import numpy as np
import time

LOWER_ORANGE = (5, 100, 100)
UPPER_ORANGE = (25, 255, 255)
MIN_AREA = 1500
_KERNEL = np.ones((5, 5), np.uint8)

class ObjectDetectionTrigger:
    def __init__(self, cam_index=0, width=1280, height=720, stability_time=1.5):
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {cam_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.last_center = None
        self.last_seen_time = None
        self.stability_time = stability_time  # seconds
        self.detection_triggered = False

    def _detect_orange(self, frame):
        """Returns (cx, cy, short_w) if an orange rectangle is found, else None."""
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
        (cx, cy), (w, h), _ = rect
        short_w = float(min(w, h))
        return int(cx), int(cy), short_w

    def run(self, visualize=True):
        """Run the detection loop until stable orange object is detected."""
        print("Starting detection loop. Waiting for stable object...")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            detection = self._detect_orange(frame)
            now = time.time()

            if detection:
                cx, cy, _ = detection

                if self.last_center is None:
                    self.last_center = (cx, cy)
                    self.last_seen_time = now
                else:
                    # check movement distance
                    dx = abs(cx - self.last_center[0])
                    dy = abs(cy - self.last_center[1])
                    dist = (dx ** 2 + dy ** 2) ** 0.5

                    if dist < 10:  # within 10 pixels = stable
                        if now - self.last_seen_time > self.stability_time:
                            print("✅ Object held steady for 1.5s — triggering robot grasp.")
                            self.detection_triggered = True
                            break
                    else:
                        # movement detected, reset timer
                        self.last_center = (cx, cy)
                        self.last_seen_time = now

                # draw detection
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(frame, "Tracking...", (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                self.last_center = None
                self.last_seen_time = None

            if visualize:
                cv2.imshow("Orange Object Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows()
        return self.detection_triggered
