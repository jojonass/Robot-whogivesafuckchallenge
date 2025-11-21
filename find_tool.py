import cv2
import numpy as np

LOWER_ORANGE = (5, 100, 100)
UPPER_ORANGE = (25, 255, 255)
MIN_AREA = 1500
_KERNEL = np.ones((5, 5), np.uint8)

_cap = None
_latest_frame = None  # always holds the newest webcam frame


def init_camera(index=0, width=1280, height=720):
    """Start webcam and begin grabbing frames."""
    global _cap
    _cap = cv2.VideoCapture(index)
    if not _cap.isOpened():
        raise RuntimeError(f"Could not open camera index {index}")
    _cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


def update_stream():
    """Call this regularly; updates _latest_frame. No detection here."""
    global _latest_frame
    if _cap is None:
        return

    ok, frame = _cap.read()
    if ok:
        _latest_frame = frame


def detect_once(save_path="last_detection.png"):
    """
    Run detection ONLY on the latest frame.
    - draws center point (and rectangle) into the image
    - saves the annotated image to save_path
    - returns (cx, cy, short_width) or None
    """
    if _latest_frame is None:
        return None

    frame = _latest_frame.copy()

    # --- detection ---
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
        # optionally save the raw frame anyway:
        cv2.imwrite(save_path, frame)
        return None

    rect = cv2.minAreaRect(best)
    (cx, cy), (w, h), _ = rect
    short_w = float(min(w, h))

    cx_i, cy_i = int(round(cx)), int(round(cy))

    # --- drawing ---
    # center dot
    cv2.circle(frame, (cx_i, cy_i), 6, (255, 0, 0), -1)

    # rectangle outline
    box = cv2.boxPoints(rect)
    box = box.astype(int)
    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

    # optional: show short side as a line
    edges = [np.linalg.norm(box[i] - box[(i + 1) % 4]) for i in range(4)]
    short_edge_i = int(np.argmin(edges))
    p1, p2 = box[short_edge_i], box[(short_edge_i + 1) % 4]
    opp_edge = (short_edge_i + 2) % 4
    p3, p4 = box[opp_edge], box[(opp_edge + 1) % 4]
    mid1 = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
    mid2 = ((p3[0] + p4[0]) // 2, (p3[1] + p4[1]) // 2)
    cv2.line(frame, mid1, mid2, (0, 255, 0), 2)

    # save annotated image
    cv2.imwrite(save_path, frame)

    return cx_i, cy_i, short_w


def release_camera():
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
