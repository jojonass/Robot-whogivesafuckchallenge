import cv2
import time
import os

# -----------------------------
# Global Flags and Config
# -----------------------------
ready_clicked = False
robot_continue = False
LOGFILE = "slide_log.txt"
FONT = cv2.FONT_HERSHEY_SIMPLEX
BUTTON_RECT = (250, 400, 550, 470)

# -----------------------------
# Logging
# -----------------------------
def log_event(message):
    t = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOGFILE, "a") as f:
        f.write(f"[{t}] {message}\n")

# -----------------------------
# Mouse callback for YES button
# -----------------------------
def mouse_callback(event, x, y, flags, param):
    global ready_clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        x1, y1, x2, y2 = BUTTON_RECT
        if x1 <= x <= x2 and y1 <= y <= y2:
            ready_clicked = True


# -----------------------------
# Slide Viewer
# -----------------------------
import screeninfo
# ---------- Modified SlideViewer ----------
import cv2
import numpy as np
import screeninfo
from datetime import datetime

class SlideViewer:
    def __init__(self, scale=0.95):
        self.window_name = "Instructions"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        monitor = screeninfo.get_monitors()[0]
        self.screen_width = int(monitor.width * scale)
        self.screen_height = int(monitor.height * scale)
        cv2.resizeWindow(self.window_name, self.screen_width, self.screen_height)
        x = (monitor.width - self.screen_width) // 2
        y = (monitor.height - self.screen_height) // 2
        cv2.moveWindow(self.window_name, x, y)

    def prepare_slide_image(self, slide):
        """Return a ready-to-show image."""
        if slide.get("text_only", False):
            img = np.full((self.screen_height, self.screen_width, 3), 200, dtype=np.uint8)
            # Draw title
            cv2.putText(img, slide["title"], (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,0), 3)
            # Draw text
            y0 = 160
            dy = 35
            for i, line in enumerate(slide["text"].split("\n")):
                y = y0 + i*dy
                cv2.putText(img, line, (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0), 2)
            return img
        else:
            img = cv2.imread(slide["image"])
            return cv2.resize(img, (self.screen_width, self.screen_height))


    def advance_from_robot(self):
        global robot_continue
        robot_continue = True




# Folder containing exported PNG slides (relative to your Python code)
slide_folder = "Slides"

# Get all PNG files in the folder, sorted by name
import re

def numeric_sort(f):
    match = re.search(r'(\d+)', f)
    return int(match.group(1)) if match else 0

slide_files = sorted(
    [f for f in os.listdir(slide_folder) if f.lower().endswith(".png")],
    key=numeric_sort
)

instruction_slides = [{"text_only": False, "image": os.path.join(slide_folder, f)} for f in slide_files]






import os
import cv2





'''

instruction_slides = [
    # Slide 0: Standby, robot-controlled
    {
        "text_only": True,
        "title": "Standby",
        "text": "Robot calibration is occurring.\nPlease wait..."
    },
    # Slide 1: Look at robot

    {
        "text_only": True,
        "title": "Look at Robot",
        "text": "Please look at the robot before starting the operations. As soon robot starts look for instructions"

    },

    # Slide 2: M3 operation
    {
        "text_only": True,
        "title": " M4",
        "text": "Now performing operations with  M3"
    },

    # Slide 4: M4 operation
    {
        "text_only": True,
        "title": " M3",
        "text": "Please wait for robot to drop tool off"
    },

    # Slide 6: Thank you slide, text-only
    {
        "text_only": True,
        "title": "Thank You!",
        "text": "The instruction routine is complete."
    },
]

'''