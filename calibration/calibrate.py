import cv2
import numpy as np
import glob
import os

CHECKERBOARD = (7, 10)   # (cols, rows) of INNER corners
SQUARE_SIZE = 0.025      # 25 mm in meters
OUT_FILE = "camera_scales.npz"
IMAGE_DIR = "../calib_images/*.jpg"

# >>> SET: distance from camera to the robot working plane (meters)
Z_WORKPLANE_M = 0.50   # example
# <<<

def main():
    objpoints = []
    imgpoints = []

    cols, rows = CHECKERBOARD
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    image_files = sorted(glob.glob(IMAGE_DIR))
    if not image_files:
        print(f"No images found for {IMAGE_DIR}")
        return

    print(f"Found {len(image_files)} calibration images")

    gray_shape = None

    for fname in image_files:
        img = cv2.imread(fname)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]  # (W, H)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)
        if not ret:
            print(f"Checkerboard not detected in {fname}")
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001)
        corners_sub = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria
        )

        objpoints.append(objp.copy())
        imgpoints.append(corners_sub)

        print(f"Used: {os.path.basename(fname)}")

    if len(objpoints) < 5:
        print(f"Not enough valid images: {len(objpoints)} (need ~5+)")

    # --- Intrinsic calibration ---
    ret, K, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, None, None
    )

    print("\nCalibration complete")
    print("Camera matrix K:\n", K)
    print("Distortion coefficients:\n", distCoeffs.ravel())

    W, H = gray_shape
    fx = K[0, 0]
    fy = K[1, 1]

    # --- Only what you need: mm/px at work plane ---
    scale_x_mm_per_px = (Z_WORKPLANE_M / fx) * 1000.0
    scale_y_mm_per_px = (Z_WORKPLANE_M / fy) * 1000.0

    print("\n--- SCALE (use these in your robot code) ---")
    print(f"scale_x_mm_per_px = {scale_x_mm_per_px:.6f}")
    print(f"scale_y_mm_per_px = {scale_y_mm_per_px:.6f}")

    # Save only the essentials
    np.savez(
        OUT_FILE,
        K=K,
        distCoeffs=distCoeffs,
        image_width=W,
        image_height=H,
        Z_workplane_m=Z_WORKPLANE_M,
        scale_x_mm_per_px=scale_x_mm_per_px,
        scale_y_mm_per_px=scale_y_mm_per_px
    )

    print(f"\nSaved to {OUT_FILE}")

if __name__ == "__main__":
    main()
