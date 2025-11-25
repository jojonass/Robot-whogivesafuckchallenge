# test_obj_recog.py

import cv2
import numpy as np
import os
import time

from object_detection_trigger import ObjectDetectionTrigger


def main():
    # We only want detection utilities here, no internal loop:
    detector = ObjectDetectionTrigger(
        cam_index=1,
        width=None,
        height=None,
        stability_time=1.5,
        timeout=0.0,  # no internal loop, no camera opened inside the class
    )

    cam_index = 1
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {cam_index}")

    # Optional: set a resolution manually, or omit these two lines
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    out_dir = "debug_captures"
    os.makedirs(out_dir, exist_ok=True)

    print("[test_trigger_debug]")
    print("  Live preview from camera", cam_index)
    print("  Press SPACE to save annotated image")
    print("  Press q to quit")

    shot_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[test_trigger_debug] Failed to grab frame")
            break

        mean_intensity = frame.mean()
        vis = frame.copy()

        # Run rectangle detection
        detection = detector._detect_orange(frame)
        now = time.time()

        lines = []
        lines.append(f"mean={mean_intensity:.1f}")

        if detection is not None:
            u, v = detection

            # draw detection marker
            cv2.drawMarker(
                vis,
                (u, v),
                (255, 0, 0),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=20,
                thickness=2,
                line_type=cv2.LINE_AA,
            )

            # stability logic (same as in the internal loop, but we keep running)
            if detector.last_center is None:
                detector.last_center = (u, v)
                detector.last_seen_time = now
            else:
                dx = abs(u - detector.last_center[0])
                dy = abs(v - detector.last_center[1])
                dist = (dx ** 2 + dy ** 2) ** 0.5

                if dist < 10:
                    # within 10 px, candidate for stability
                    if detector.last_seen_time is None:
                        detector.last_seen_time = now
                    else:
                        held_time = now - detector.last_seen_time
                        if held_time >= detector.stability_time:
                            detector.detection_triggered = True
                else:
                    detector.last_center = (u, v)
                    detector.last_seen_time = now
                    detector.detection_triggered = False

            lines.append(f"u,v=({u},{v})")
            if detector.last_seen_time is not None:
                held_time = now - detector.last_seen_time
            else:
                held_time = 0.0
            lines.append(f"held={held_time:.2f}s")
            lines.append(f"stable_detected={detector.detection_triggered}")
        else:
            lines.append("No object detected")
            detector.last_center = None
            detector.last_seen_time = None
            detector.detection_triggered = False

        # Draw overlay text
        y0 = 25
        dy = 22
        for i, text in enumerate(lines):
            y = y0 + i * dy
            # black outline
            cv2.putText(
                vis,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            # white text
            cv2.putText(
                vis,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow("trigger debug", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("[test_trigger_debug] Quitting.")
            break
        elif key == ord(" "):
            ts = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(out_dir, f"debug_{ts}_{shot_idx:03d}.png")
            cv2.imwrite(filename, vis)
            print(f"  Saved annotated image to: {filename}")
            shot_idx += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
