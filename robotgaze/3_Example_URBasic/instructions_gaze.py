import os
import cv2
import numpy as np
import screeninfo

class SlideViewer:
    """Simple slide viewer for PNG images with OpenCV."""

    def __init__(self, scale: float = 0.95):
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

        if slide.get("text_only", False):
            img = np.full((self.screen_height, self.screen_width, 3), 200, dtype=np.uint8)

            cv2.putText(img, slide.get("title", ""), (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3)

            y0 = 160
            for i, line in enumerate(slide.get("text", "").split("\n")):
                cv2.putText(img, line, (30, y0 + i * 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)

            return img

        img = cv2.imread(slide["image"])
        if img is None:
            img = np.full((self.screen_height, self.screen_width, 3), 100, dtype=np.uint8)
            cv2.putText(img, "SLIDE NOT FOUND", (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            return img

        return cv2.resize(img, (self.screen_width, self.screen_height))


# =========================
# Load PNG slides from ./Slides
# =========================

slide_folder = "Slides/slides_gaze"

# Create dict from Folie1–Folie12 filenames if they exist
instruction_slides = {}

for i in range(1, 13):
    filename = f"Folie{i}.png"
    full_path = os.path.join(slide_folder, filename)
    if os.path.exists(full_path):
        instruction_slides[f"FOLIE{i}"] = {"text_only": False, "image": full_path}



SLIDE_STANDBY           = instruction_slides["FOLIE1"]
SLIDE_LOOK_AT_ROBOT     = instruction_slides["FOLIE2"]
SLIDE_PICKUP_M4         = instruction_slides["FOLIE3"]
SLIDE_PLACE_M4          = instruction_slides["FOLIE4"]
SLIDE_PICKUP_M5         = instruction_slides["FOLIE5"]
SLIDE_PLACE_M5          = instruction_slides["FOLIE6"]
SLIDE_PICKUP_M3         = instruction_slides["FOLIE7"]
SLIDE_PLACE_M3          = instruction_slides["FOLIE8"]
SLIDE_TRUST_TEST        = instruction_slides["FOLIE9"]
SLIDE_RETURN_HOME       = instruction_slides["FOLIE10"]
SLIDE_BATTERY           = instruction_slides["FOLIE11"]
SLIDE_THANK_YOU         = instruction_slides["FOLIE12"]
