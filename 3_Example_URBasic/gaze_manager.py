import threading
import time
import cv2
import numpy as np

# Assumed imports from files provided previously:
from camera import CameraManager
from eye_contact_detector import EyeContactDetector

# --- CONFIGURATION ---
CAMERA_ID = 0
GAZE_HOLD_THRESHOLD = 0.5
GAZE_MEMORY_WINDOW = 4.0

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

    def __init__(self, cam_id=CAMERA_ID):
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

    def get_gaze_state(self):
        with self.lock:
            return self.gaze_state

    # --- UPDATED set_active METHOD (Controls both tracking and window closure) ---
    def set_active(self, status: bool, close_window: bool = False):
        """
        Sets the tracking status and optionally closes the window.
        status: If True, gaze tracking/memory is active.
        close_window: If True, specifically destroys the CV window.
        """
        with self.lock:
            self.is_active = status

            # --- CRITICAL FIX: Close window logic remains here ---
            if close_window and cv2.getWindowProperty("Gaze Feedback", cv2.WND_PROP_VISIBLE) >= 1:
                # Check if the window is visible/exists before trying to destroy it
                cv2.destroyWindow("Gaze Feedback")

            print(f"[{self.name}] Tracking set to {'ACTIVE' if status else 'INACTIVE'}")

    def recalibrate(self):
        with self.lock:
            if self.detector:
                print(f"\n[{self.name}] ** EXTERNAL RECALIBRATION TRIGGERED **")
                self.is_calibrated = False
                self.detector.start_calibration(self.cam_id)
            else:
                print(f"\n[{self.name}] Detector not initialized yet. Skipping recalibration.")

    def stop(self):
        """Cleanly stops the worker thread and its internal camera streams."""
        print(f"[{self.name}] Received stop signal. Setting running=False.")
        self.running = False
        # Close the window immediately if it exists to ensure the thread can exit cleanly
        self.set_active(False, close_window=True)

    def run(self):
        print(f"[{self.name}] Starting Gaze Worker...")

        self.cam_manager = CameraManager(camera_ids=[self.cam_id])
        self.detector = EyeContactDetector(self.cam_manager)

        time.sleep(1)
        if self.cam_id not in self.cam_manager.cameras:
            print(f"[{self.name}] ERROR: Camera {self.cam_id} failed to initialize. Exiting worker.")
            self.running = False
            return

        # THIS IS THE ONE AND ONLY CALIBRATION TRIGGER
        self.detector.start_calibration(self.cam_id)
        # Set to True so the main loop starts in calibration mode
        self.set_active(True)

        try:
            while self.running:

                ret, frame = self.cam_manager.get_frame(self.cam_id)

                if not ret:
                    time.sleep(0.01)
                    continue

                # --- Calibration Phase ---
                if self.detector.needs_calibration.get(self.cam_id, False):
                    self.is_calibrated = False

                    # Window Management ONLY during Calibration
                    if cv2.getWindowProperty("Gaze Feedback", cv2.WND_PROP_VISIBLE) < 1:
                        cv2.namedWindow("Gaze Feedback", cv2.WINDOW_AUTOSIZE)

                    # is_active=True ensures visualization is drawn during calibration
                    gaze_data = self.detector.detect_and_draw(self.cam_id, frame, is_active=True)

                    elapsed = time.time() - self.detector.calibration_start_time.get(self.cam_id, time.time())
                    remaining = max(0, self.detector.CALIBRATION_TIME - elapsed)

                    text = f"CALIBRATING ({remaining:.1f}s) - Look Straight"
                    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.imshow("Gaze Feedback", frame)

                    # Check if calibration just completed
                    if not self.detector.needs_calibration.get(self.cam_id, False):
                        self.is_calibrated = True
                        center_x = self.detector.calibrated_data.get(self.cam_id)
                        print(f"[{self.name}] Calibration complete! Center X: {center_x:.3f}")

                        # --- CRITICAL FIX SECTION ---

                        # 1. Close the window immediately if the config is False
                        if not KEEP_WINDOW_ACTIVE_AFTER_CALIB:
                            # Keep tracking active (status=True), but close the window (close_window=True)
                            self.set_active(True, close_window=True)
                            print(f"[{self.name}] Window visualization explicitly closed by config.")
                        else:
                            # 2. If config is True, keep tracking active (window stays open)
                            self.set_active(True)

                            # --- Normal Operation (Tracking) ---
                else:
                    # Perform detection only if tracking is active
                    if self.is_active:
                        # is_active=True here only controls internal drawing in detector, not window visibility
                        gaze_data = self.detector.detect_and_draw(self.cam_id, frame, is_active=True)

                        is_detected = gaze_data['is_detected']
                        current_zone = gaze_data['zone_label']

                        # --- Gaze Memory Update ---
                        self.gaze_memory.update(is_detected, current_zone)

                        with self.lock:
                            self.gaze_state = current_zone

                    # ❌ CRITICAL CHANGE: NO CV2.IMSHOW or CV2.NAMEDWINDOW calls here.

                # This call is still necessary to process CV2 window events for the brief time it's open
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break

                time.sleep(0.01)

        except Exception as e:
            print(f"[{self.name}] An error occurred in the main loop: {e}")
            self.running = False

        finally:
            print(f"[{self.name}] Stopping Gaze Worker thread...")
            # This ensures any remaining windows are destroyed on shutdown
            cv2.destroyAllWindows()
            if self.cam_manager:
                self.cam_manager.stop_all()


def wait_for_gaze_command(worker: GazeWorker, required_zone: str):
    GAZE_MEMORY_WINDOW = 10.0
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
