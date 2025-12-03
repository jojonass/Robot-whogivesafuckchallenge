import cv2
import os
import re
import screeninfo

class SlideViewer:
    def __init__(self, slide_folder="Slides_button", scale=0.95):
        self.window_name = "Instructions"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Get monitor info
        monitor = screeninfo.get_monitors()[0]
        self.screen_width = int(monitor.width * scale)
        self.screen_height = int(monitor.height * scale)
        cv2.resizeWindow(self.window_name, self.screen_width, self.screen_height)
        x = (monitor.width - self.screen_width) // 2
        y = (monitor.height - self.screen_height) // 2
        cv2.moveWindow(self.window_name, x, y)

        # Load slides from folder
        self.slides = self.load_slides(slide_folder)

    def load_slides(self, folder):
        files = [f for f in os.listdir(folder) if f.lower().endswith(".png")]

        # Sort numerically if filenames contain numbers
        def numeric_sort(f):
            match = re.search(r'(\d+)', f)
            return int(match.group(1)) if match else 0

        files = sorted(files, key=numeric_sort)
        slides = [{"text_only": False, "image": os.path.join(folder, f)} for f in files]

        if not slides:
            raise FileNotFoundError(f"No PNG slides found in folder '{folder}'")
        return slides

    def prepare_slide_image(self, slide_index):
        """Return the slide image ready to show"""
        if slide_index < 0 or slide_index >= len(self.slides):
            slide_index = 0
        slide = self.slides[slide_index]
        img = cv2.imread(slide["image"])
        return cv2.resize(img, (self.screen_width, self.screen_height))


# ----------------------------
# Initialize SlideViewer
# ----------------------------
slide_viewer = SlideViewer(slide_folder="Slides_button")
instruction_slides = slide_viewer.slides  # use the actual slide dictionaries
