import cv2
import threading
import time


class CameraStream:
    """Handles individual camera stream in a thread."""

    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        print(f"Initializing camera stream for camera_id={camera_id}")

        # Using a specialized backend for better stability if available (optional, but sometimes helps)
        # self.cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW) # Use DSHOW on Windows if available
        self.cap = cv2.VideoCapture(camera_id)

        self.ret = False
        self.frame = None

        # Check if the camera opened successfully
        if not self.cap.isOpened():
            print(f"Warning: Camera {camera_id} could not be opened.")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        self.stopped = False

        # Start the thread only if the camera opened successfully
        if self.cap.isOpened():
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        """Thread worker to continuously read frames."""
        # Poll the camera for new frames
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame
            else:
                # If reading fails, pause briefly to avoid high CPU usage on a dead stream
                time.sleep(0.01)

    def read(self):
        """Returns the latest frame and success status."""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            return False, None

        with self.lock:
            # Return a copy to prevent the main thread modifying the frame while the camera thread updates it
            return self.ret, self.frame.copy() if self.ret and self.frame is not None else (False, None)

    def stop(self):
        """Stops the thread and releases the camera resource."""
        self.stopped = True
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)  # Wait for thread to finish
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()


class CameraManager:
    """Manages multiple camera streams simultaneously."""

    def __init__(self, camera_ids=[0]):
        self.cameras = {}
        # Initialize the camera streams for the given camera IDs
        self._initialize_cameras(camera_ids)

    def _initialize_cameras(self, camera_ids):
        """Initialize CameraStream objects for the provided camera IDs."""
        for cam_id in camera_ids:
            if cam_id not in self.cameras:
                try:
                    stream = CameraStream(cam_id)
                    # Only store the stream if it was successfully initialized (cap is opened)
                    if hasattr(stream, 'cap') and stream.cap.isOpened():
                        self.cameras[cam_id] = stream
                    else:
                        print(f"Skipping camera {cam_id} due to initialization failure.")
                except Exception as e:
                    print(f"Failed to initialize camera {cam_id}: {e}")

    def get_frame(self, cam_id):
        """Get the latest frame from the specified camera."""
        if cam_id not in self.cameras:
            # If the requested cam_id is not yet initialized, try to initialize it
            self._initialize_cameras([cam_id])
            if cam_id not in self.cameras:
                # If still not initialized, raise an error
                raise ValueError(f"Camera {cam_id} not available or failed to initialize.")

        # Read frame from the active stream
        return self.cameras[cam_id].read()

    def stop_all(self):
        """Stop all camera streams and release resources."""
        print("Stopping all camera streams...")
        for cam in self.cameras.values():
            cam.stop()
        self.cameras = {}  # Clear the dictionary



class CameraRecorder:
    """
    Handles recording from CameraManager streams.
    Can record multiple cameras simultaneously.
    """

    def __init__(self, camera_manager: CameraManager):
        self.camera_manager = camera_manager
        self.recorders = {}  # cam_id -> (VideoWriter, filename)
        self.recording_flags = {}  # cam_id -> bool

    def start_recording(self, cam_id, filename, fps=30, resolution=(640, 480)):
        """Start recording a specific camera."""
        if cam_id not in self.camera_manager.cameras:
            raise ValueError(f"Camera {cam_id} not available for recording.")

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(filename, fourcc, fps, resolution)
        self.recorders[cam_id] = (writer, filename)
        self.recording_flags[cam_id] = True
        print(f"Started recording camera {cam_id} to {filename}")

        # Start recording thread
        threading.Thread(target=self._record_loop, args=(cam_id,), daemon=True).start()

    def _record_loop(self, cam_id):
        """Continuously write frames to file while recording."""
        writer, _ = self.recorders[cam_id]
        while self.recording_flags.get(cam_id, False):
            ret, frame = self.camera_manager.get_frame(cam_id)
            if ret and frame is not None:
                writer.write(frame)
            else:
                # Wait briefly if no frame
                time.sleep(0.01)
        writer.release()
        print(f"Stopped recording camera {cam_id}")

    def stop_recording(self, cam_id):
        """Stop recording a specific camera."""
        if cam_id in self.recording_flags:
            self.recording_flags[cam_id] = False

    def stop_all(self):
        """Stop all recordings."""
        for cam_id in list(self.recording_flags.keys()):
            self.stop_recording(cam_id)
