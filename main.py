import tkinter as tk
from utility_functions import *
from eye_contact_detector import EyeContactDetector
from Action_chunks import playback_chunk, execute_action
from Slides import *




# Conceptual Robot Logic
# ... (your existing imports)
STATE_FILE = "robot_state.json"



state = read_global_state()

"""
# start

root = tk.Tk()
app = EyeContactApp(root)
root.mainloop()

# wait for calibration delay 5 seconds, don't let window pop up(change later)




# Start

if check.position.home() == True
    state = read_global_state()
    update_robot_state(new_slide_id=1)
    if state["human_input_ready"] == True:
    update_robot_state(new_slide_id=2)
    chunk = json.search(central_tool_location)
    play_back(chunk)


# Get tool
state = read_global_state()
zone_value, proximity = app.update_frame()


if state["current_slide_id"] = 2  and (zone_value = "" and proximity = "")
    chunk  = json.search(go_screw_driver)
    play_back(chunk)
    grip.screwdriver()
    chunk = json.search(go_tool_square)
    play_back(chunk)
    ungrip.screwdriver()
    chunk = json.search(go_tool_view)
    play_back(chunk)
    

state = read_global_state()
zone_value, proximity = app.update_frame()
wrist_camera = gripper.camera_update()

# Place back  tool 
if state["current_slide_id"] = 2  and (zone_value = "" and proximity = "")  and  wrist_camera = "screw_driver" 
    
    update_robot_state(new_slide_id=3)
    chunk = json.search(go_tool_cup)
    play_back(chunk)
    grip(screw_driver)
    chunk = json.search(go_screw_driver)
    play_back(chunk)
    chunk = json.search(place_screw_driver)
    execute(chunk)
    grip(place)
    chunk = json.search(central_tool_location)
    play_back(chunk)
    
    
    
    
    
# Finish 

update_robot_state(new_slide_id=n)

if gui_instruction = "Finish" and gui_input = "finish" 
    chunk = json.search(home)
    play_back(chunk)
    check_position.home()
    







"""

