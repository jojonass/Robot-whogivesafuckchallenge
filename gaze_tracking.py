import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
from eye_contact_detector import EyeContactDetector
from camera import CameraManager
import time

# ----------------- CONFIG -----------------
FORCE_CAMERA_ID = 0      # Set to None to detect cameras automatically #FORCE_CAMERA_IDS = [1, 2]  webcam + wrist for example
AUTO_HIDE_AFTER_CALIB = True # Set to False to keep GUI visible after calibration
# -----------------------------------------

class EyeContactApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Camera Gaze Tracking")

        # --- Camera selection ---
        if FORCE_CAMERA_ID is not None:
            # Force specific camera
            # --- Camera selection ---
            self.camera_ids = FORCE_CAMERA_ID
            self.active_cam = self.camera_ids[0]  # keep first as reference
            print(f"Using cameras: {self.camera_ids}")

        else:
            # Detect available cameras
            self.camera_ids = self.detect_cameras(max_cams=5)
            if not self.camera_ids:
                self.camera_ids = [0]
                print("Warning: No cameras detected. Defaulting to camera ID 0.")
            self.active_cam = self.camera_ids[0]
            print(f"Detected cameras: {self.camera_ids}. Starting with ID: {self.active_cam}")

        # Initialize CameraManager
        self.camera_manager = CameraManager(camera_ids=self.camera_ids)

        # Initialize the multi-camera detector
        self.detector = EyeContactDetector(camera_manager=self.camera_manager)

        # --- UI Elements ---
        # --- UI Elements ---
        self.label = tk.Label(root, text="Dual Camera Gaze Tracking", font=("Helvetica", 20))
        self.label.pack(pady=5)

        # Two canvas panels side by side
        canvas_frame = tk.Frame(root)
        canvas_frame.pack()

        self.canvas_width = 480
        self.canvas_height = 360
        self.canvases = {}
        self.info_labels = {}

        for i, cam_id in enumerate(self.camera_ids):
            tk.Label(canvas_frame, text=f"Camera {cam_id}", font=("Helvetica", 14)).grid(row=0, column=i)
            canvas = tk.Canvas(canvas_frame, width=self.canvas_width, height=self.canvas_height, bg="black")
            canvas.grid(row=1, column=i, padx=5, pady=5)
            self.canvases[cam_id] = canvas
            info = tk.Label(canvas_frame, text=f"Zone: -- | Proximity: --", font=("Helvetica", 12))
            info.grid(row=2, column=i)
            self.info_labels[cam_id] = info

        # Control buttons frame
        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=10)

        self.calibrate_btn = ttk.Button(btn_frame, text="Calibrate", command=self.calibrate)
        self.calibrate_btn.grid(row=0, column=0, padx=10)

        self.quit_btn = ttk.Button(btn_frame, text="Quit", command=self.quit_program)
        self.quit_btn.grid(row=0, column=1, padx=10)

        # Recording toggle button
        self.recording = False
        self.out = None
        self.record_btn = ttk.Button(btn_frame, text="Start Recording", command=self.toggle_recording)
        self.record_btn.grid(row=0, column=4, padx=10)

        cam_label = tk.Label(btn_frame, text="Active Camera:")
        cam_label.grid(row=0, column=2, padx=(20, 5))

        self.camera_selector = ttk.Combobox(btn_frame, values=self.camera_ids, width=5, state="readonly")
        self.camera_selector.current(0)
        if FORCE_CAMERA_ID is not None:
            self.camera_selector.config(state="disabled")  # Disable if camera is forced
        self.camera_selector.grid(row=0, column=3)
        self.camera_selector.bind("<<ComboboxSelected>>", self.change_camera)

        # Start updating frames
        self.calibrate()  # Initial calibration
        self.update_frame()

    # ----------------- CAMERA DETECTION -----------------
    def detect_cameras(self, max_cams=5):
        """Detects available camera IDs."""
        available_cams = []
        for i in range(max_cams):
            cap = cv2.VideoCapture(i)
            if cap.isOpened() and cap.read()[0]:
                available_cams.append(i)
                cap.release()
        return available_cams

    def change_camera(self, event=None):
        """Switches the active camera from combobox."""
        new_cam_id_str = self.camera_selector.get()
        try:
            new_cam_id = int(new_cam_id_str)
        except ValueError:
            print(f"Invalid camera ID: {new_cam_id_str}")
            return
        self.active_cam = new_cam_id
        print(f"Switched active camera to ID: {new_cam_id}")
        self.calibrate()

    # ----------------- CALIBRATION -----------------
    def calibrate(self):
        """Start calibration for the active camera."""
        self.detector.calibrate(self.active_cam)
        self.label.config(text="Calibrating...", fg="orange")
        print(f"Calibration started for camera {self.active_cam}")

        # Schedule GUI hide after calibration
        if AUTO_HIDE_AFTER_CALIB:
            calibration_time = self.detector.CALIBRATION_TIME
            self.root.after(int(calibration_time*1000), self.hide_window)

    def hide_window(self):
        """Hide the GUI window."""
        self.root.withdraw()
        print("GUI hidden. Eye contact detection continues in background.")

    # ----------------- RECORDING -----------------
    def toggle_recording(self):
        """Start/stop recording video."""
        if not self.recording:
            # Start recording
            filename = f"camera_{self.active_cam}_{int(time.time())}.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(filename, fourcc, 20.0, (self.canvas_width, self.canvas_height))
            self.recording = True
            self.record_btn.config(text="Stop Recording")
            print(f"Recording started: {filename}")
        else:
            # Stop recording
            self.recording = False
            if self.out:
                self.out.release()
                self.out = None
            self.record_btn.config(text="Start Recording")
            print("Recording stopped")

    # ----------------- MAIN LOOP -----------------
    def update_frame(self):
        """Update loop for both camera frames."""
        for cam_id in self.camera_ids:
            frame, eye_contact, zone, proximity = self.detector.detect_eye_contact(cam_id)

            # Update per-camera info
            if self.detector.needs_calibration.get(cam_id, True):
                self.info_labels[cam_id].config(text=f"Cam {cam_id}: Calibrating...", fg="orange")
            elif eye_contact:
                self.info_labels[cam_id].config(text=f"Cam {cam_id}: Eye Contact ({zone})", fg="green")
            else:
                self.info_labels[cam_id].config(text=f"Cam {cam_id}: No Eye Contact", fg="red")

            # Convert and display
            if frame is not None:
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(img)
                img = img.resize((self.canvas_width, self.canvas_height), Image.LANCZOS)
                img_tk = ImageTk.PhotoImage(image=img)
                self.canvases[cam_id].create_image(0, 0, anchor="nw", image=img_tk)
                self.canvases[cam_id].img_tk = img_tk  # keep reference

        self.root.after(10, self.update_frame)

    # ----------------- QUIT -----------------
    def quit_program(self):
        """Safely quit the application."""
        print("Stopping all cameras and detector...")
        if self.out:
            self.out.release()
        self.camera_manager.stop_all()
        self.detector.release()
        self.root.destroy()


# --- Gaze Memory Tracker ---
class GazeMemory:
    def __init__(self, hold_threshold=1.0, memory_window=10.0):
        """
        hold_threshold  : how long gaze must last (seconds)
        memory_window   : how long after gaze we still consider it valid (seconds)
        """
        self.hold_threshold = hold_threshold
        self.memory_window = memory_window
        self.last_hold_time = None
        self.last_zone = None
        self._contact_start = None
        self._current_zone = None

    def update(self, eye_contact_active: bool, zone_value: str):
        now = time.time()

        if eye_contact_active and zone_value:
            # start or continue counting sustained contact in a zone
            if self._contact_start is None or zone_value != self._current_zone:
                self._contact_start = now
                self._current_zone = zone_value
            elif now - self._contact_start >= self.hold_threshold:
                self.last_hold_time = now
                self.last_zone = zone_value
        else:
            # reset if no gaze
            self._contact_start = None
            self._current_zone = None

    def recently_valid(self, required_zone=None):
        """Return True if a valid gaze hold occurred in the right zone within memory_window."""
        if self.last_hold_time is None:
            return False
        if required_zone and self.last_zone != required_zone:
            return False
        return (time.time() - self.last_hold_time) <= self.memory_window


# ----------------- RUN -----------------
if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = EyeContactApp(root)
        root.mainloop()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        if 'app' in locals():
            app.quit_program()
