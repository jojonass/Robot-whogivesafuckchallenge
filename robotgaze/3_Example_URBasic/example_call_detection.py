from object_detection_trigger import ObjectDetectionTrigger

print("Waiting for stable object...")

# This call will BLOCK until stable detection OR timeout
detector = ObjectDetectionTrigger(
    cam_index=1,
    stability_time=1.5,
    timeout=9999.0   # wait effectively "forever"
)

if detector:
    print("Stable object detected. Moving robot...")
    robot.movel(pose=pose_tool_square_approach, a=acc, v=vel)
    robot.movel(pose=pose_A_approach, a=acc, v=vel)
else:
    print("No detection (timeout).")
