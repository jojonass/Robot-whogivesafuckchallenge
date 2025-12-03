import threading
import time
import cv2
import numpy as np

# Assumed imports from files provided previously:
from camera import CameraManager
from eye_contact_detector import EyeContactDetector

# --- CONFIGURATION ---
CAMERA_ID = 0
GAZE_HOLD_THRESHOLD = 0
GAZE_MEMORY_WINDOW = 1.0

# 🌟 VISUALIZATION CONFIGURATION 🌟
# Set to True to keep the 'Gaze Feedback' window open and displaying zone drawings after calibration.
# Set to False to automatically close the window after calibration is finished.
KEEP_WINDOW_ACTIVE_AFTER_CALIB = False


# ---------------------


# --- Gaze Memory Tracker ---
class GazeMemory:

    def __init__(self, hold_threshold=GAZE_HOLD_THRESHOLD, memory_window=GAZE_MEMORY_WINDOW):
        self.hold_threshold = hold_threshold
        self.memory_window = memory_window
        self.last_hold_time = None
        self.last_zone = None
        self._contact_start = None
        self._current_zone = None
        self.lock = threading.Lock()


    def update(self, is_detected: bool, zone_value: str):
        now = time.time()
        with self.lock:
            if is_detected and zone_value:
                if self._contact_start is None or zone_value != self._current_zone:
                    self._contact_start = now
                    self._current_zone = zone_value
                elif now - self._contact_start >= self.hold_threshold:
                    self.last_hold_time = now
                    self.last_zone = zone_value
            else:
                self._contact_start = None
                self._current_zone = None

    def recently_valid(self, required_zone=None):
        with self.lock:
            if self.last_hold_time is None: return False
            if required_zone and self.last_zone != required_zone: return False
            return (time.time() - self.last_hold_time) <= self.memory_window

    def clear_memory(self):
        with self.lock:
            self.last_hold_time = None
            self.last_zone = None


# -----------------------------

class GazeWorker(threading.Thread):

    def __init__(self, cam_id=CAMERA_ID, record_filename="gaze_recording.mp4", record_fps=20):
        super().__init__()
        self.cam_id = cam_id
        self.gaze_state = None
        self.running = True
        self.is_calibrated = False
        self.is_active = False
        self.lock = threading.Lock()
        self.gaze_memory = GazeMemory()
        self.cam_manager = None
        self.detector = None

        # --- Recording ---
        self.recording = True
        self.video_writer = None
        self.record_filename = record_filename
        self.record_fps = record_fps

    def set_active(self, status: bool):
        with self.lock:
            self.is_active = status
            print(f"[{self.name}] Tracking set to {'ACTIVE' if status else 'INACTIVE'}")

    def stop(self):
        print(f"[{self.name}] Stopping GazeWorker...")
        self.running = False

    def run(self):
        print(f"[{self.name}] Starting Gaze Worker...")

        self.cam_manager = CameraManager(camera_ids=[self.cam_id])
        self.detector = EyeContactDetector(self.cam_manager)
        time.sleep(1)

        if self.cam_id not in self.cam_manager.cameras:
            print(f"[{self.name}] ERROR: Camera {self.cam_id} failed to initialize. Exiting worker.")
            self.running = False
            return

        # --- Initialize VideoWriter for recording all frames ---
        width, height = self.cam_manager.get_resolution(self.cam_id)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(self.record_filename, fourcc, self.record_fps, (width, height))
        print(f"[{self.name}] Recording started → {self.record_filename}")

        # Start calibration
        self.detector.start_calibration(self.cam_id)
        self.set_active(True)

        try:
            while self.running:
                ret, frame = self.cam_manager.get_frame(self.cam_id)
                if not ret:
                    time.sleep(0.01)
                    continue

                # --- Calibration phase ---
                if self.detector.needs_calibration.get(self.cam_id, False):
                    self.is_calibrated = False
                    gaze_data = self.detector.detect_and_draw(self.cam_id, frame, is_active=True)
                    elapsed = time.time() - self.detector.calibration_start_time.get(self.cam_id, time.time())
                    remaining = max(0, self.detector.CALIBRATION_TIME - elapsed)
                    cv2.putText(frame, f"CALIBRATING ({remaining:.1f}s) - Look Straight",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                    # Show window and move to corner after calibration
                    if cv2.getWindowProperty("Gaze Feedback", cv2.WND_PROP_VISIBLE) < 1:
                        cv2.namedWindow("Gaze Feedback", cv2.WINDOW_NORMAL)
                        cv2.resizeWindow("Gaze Feedback", 1280, 960)
                        screen_width, screen_height = 1920, 1080
                        x = (screen_width - 1280) // 2
                        y = (screen_height - 960) // 2
                        cv2.moveWindow("Gaze Feedback", x, y)

                    cv2.imshow("Gaze Feedback", frame)

                    # Record frame
                    if self.video_writer:
                        self.video_writer.write(frame)

                    # Check if calibration complete
                    if not self.detector.needs_calibration.get(self.cam_id, False):
                        self.is_calibrated = True
                        print(f"[{self.name}] Calibration complete! Center X: {self.detector.calibrated_data.get(self.cam_id)}")

                        # Move window to top-left corner
                        SMALL_W, SMALL_H = 280, 210
                        POS_X, POS_Y = 10, 10
                        cv2.resizeWindow("Gaze Feedback", SMALL_W, SMALL_H)
                        cv2.moveWindow("Gaze Feedback", POS_X, POS_Y)
                        print(f"[{self.name}] Window moved to corner after calibration.")

                else:
                    # Normal tracking
                    if self.is_active:
                        gaze_data = self.detector.detect_and_draw(self.cam_id, frame, is_active=True)
                        self.gaze_memory.update(gaze_data['is_detected'], gaze_data['zone_label'])
                        with self.lock:
                            self.gaze_state = gaze_data['zone_label']

                    # Show small tracking window if enabled
                    if cv2.getWindowProperty("Gaze Feedback", cv2.WND_PROP_VISIBLE) >= 1:
                        cv2.imshow("Gaze Feedback", frame)

                    # Record all frames
                    if self.video_writer:
                        self.video_writer.write(frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break

                time.sleep(0.01)

        finally:
            print(f"[{self.name}] Shutting down Gaze Worker...")
            if self.video_writer:
                self.video_writer.release()
            cv2.destroyAllWindows()
            if self.cam_manager:
                self.cam_manager.stop_all()


def wait_for_gaze_command(worker: GazeWorker, required_zone: str):
    GAZE_MEMORY_WINDOW = 5.0
    print(f"\n[ASSEMBLY] Waiting for command: '{required_zone}' (valid for {GAZE_MEMORY_WINDOW}s after hold)...")

    while worker.running:

        # CRITICAL FIX: Ensure worker is active before checking memory.
        if not worker.is_active:
            # We don't want to stop if it's inactive, we just wait.
            # However, since we fixed it to be always active after calib, this check is mostly defensive.
            time.sleep(0.1)
            continue

        if worker.gaze_memory.recently_valid(required_zone=required_zone):
            print(f"\n[COMMAND] Gaze command '{required_zone}' confirmed (from memory)! Clearing memory slot.")
            worker.gaze_memory.clear_memory()
            return True

        if not worker.is_alive():
            return False

        time.sleep(0.1)

    return False


def wait_until_calibrated(self):
    """
    Blocks until the calibration is finished and window closed if needed.
    """
    print(f"[{self.name}] Waiting for calibration to complete...")
    while not self.is_calibrated and self.running:
        time.sleep(0.1)
    print(f"[{self.name}] Calibration completed!")
