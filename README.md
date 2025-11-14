
Robot_main.py runs the main code for the control logic with a live robot, it will go through slides, incorporate both camera feeds, and use sequential logic to move along the steps of the assembly

Test_robot_control.py runs a mock simulation with live camera feeds to test logic 

action_chunks.py interfaces with urx to let you both create motion like linear motion and closing and opening the gripper, it also lets you to record multiple actions called chunks, 
these are saved on json file that can then be played back

gaze_tracking.py creates the gui needed for live webcame feed and recording, as well as allowing the division og gaze zones that help with control logic in Robot_main.py
This also contains the feature to control the number of cameras to be used and which camera IDS as well. 

Slides.py hosts the slides that create the Gui, these are then sequenced throuhg in the main loop, and contain the instructions of the assembly


Dependencies 
python 3.12 required
numpy
mediapipe
urx
opencv-python
streamlit
streamlit_autorefresh
pillow


