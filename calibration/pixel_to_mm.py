# Example values (you'll need to adjust these based on your setup)
image_width = 640   # Image width in pixels
image_height = 480  # Image height in pixels
camera_fov_x = 200  # FOV in mm (horizontal direction, how wide the camera sees)
camera_fov_y = 150  # FOV in mm (vertical direction)

# Calculate the scale (mm per pixel)
scale_x = camera_fov_x / image_width  # mm per pixel in x-direction
scale_y = camera_fov_y / image_height  # mm per pixel in y-direction

# Example center of the image in pixels (camera center point)
cx, cy = image_width / 2, image_height / 2  # image center in pixels

# Assume TCP position is fixed (robot coordinate system)
TCP_X, TCP_Y = 500, 300  # TCP center in mm (example)

# Example object center in the image (from OpenCV object detection)
u, v = 350, 250  # Example object center in pixels

# Calculate the pixel offset from image center
delta_u = u - cx
delta_v = v - cy

# Convert pixel offset to real-world movement in mm
delta_X = delta_u * scale_x
delta_Y = delta_v * scale_y

# New TCP position (where to move the TCP)
new_TCP_X = TCP_X + delta_X
new_TCP_Y = TCP_Y + delta_Y

# Print the result (where to move the robot)
print(f"Move TCP to: X = {new_TCP_X} mm, Y = {new_TCP_Y} mm")
