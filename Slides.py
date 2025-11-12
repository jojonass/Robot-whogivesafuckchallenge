import streamlit as st
import json
import time
import os
from streamlit_autorefresh import st_autorefresh

STATE_FILE = "robot_state.json"

# --- Slides ---
SLIDES = [
    {"instruction": "System Initialization Complete. Robot ready for first task.", "type": "text"},
    {"instruction": "STEP 1: Pick up the screwdriver. Press 'I'M READY' when done.",
     "type": "image", "url": "https://placehold.co/600x400/3561aa/ffffff?text=STEP%201%0A(Pick%20Up%20Tool)"},
    {"instruction": "STEP 2: Loosen the top screw. Proceed with caution.",
     "type": "image", "url": "https://placehold.co/600x400/983c3c/ffffff?text=STEP%202%0A(Screwing%20Action)"},
    {"instruction": "STEP 3: Place the clip using pliers. Robot standing by.",
     "type": "image", "url": "https://placehold.co/600x400/3c9854/ffffff?text=STEP%203%0A(Assembly%20Clip)"},
    {"instruction": "ASSEMBLY COMPLETE. Robot returning to home position.", "type": "text"},
]

# --- Utility Functions ---
def initialize_state():
    if not os.path.exists(STATE_FILE):
        data = {"current_slide_id": 0, "human_input_ready": False}
        with open(STATE_FILE, "w") as f:
            json.dump(data, f, indent=4)
        return data
    else:
        try:
            with open(STATE_FILE, "r") as f:
                return json.load(f)
        except:
            return {"current_slide_id": 0, "human_input_ready": False}

def read_state():
    with open(STATE_FILE, "r") as f:
        return json.load(f)

def update_human_input(is_ready):
    state = read_state()
    state["human_input_ready"] = is_ready
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=4)

def update_robot_state(new_slide_id, clear_human_input=True):
    state = read_state()
    state["current_slide_id"] = new_slide_id
    if clear_human_input:
        state["human_input_ready"] = False
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=4)

# --- Streamlit GUI ---
st.set_page_config(layout="wide", page_title="HRI Assembly Guide")
st.title("🤖 Collaborative Assembly Guide")
st_autorefresh(interval=2000, key="data_refresh")

state = initialize_state()
slide_index = state["current_slide_id"]
human_ready = state["human_input_ready"]

# --- Display Slide ---
slide = SLIDES[slide_index]
st.subheader(f"Step {slide_index}")
if slide["type"] == "image" and "url" in slide:
    st.image(slide["url"], use_column_width=True)
else:
    st.info(slide["instruction"])
st.markdown(f"### {slide['instruction']}")
st.markdown("---")

# --- Human Input ---
st.subheader("👷 Human Action Control")
# Only show button for interactive steps (not first or last)
if 0 < slide_index < len(SLIDES) - 1:
    if not human_ready:
        if st.button(" I'M READY / TASK COMPLETE"):
            update_human_input(True)
            st.success("Signal sent to Robot: Human ready for next step.")
            time.sleep(0.5)
    else:
        st.info("Waiting for robot to complete this step...")
else:
    st.info("No input needed — system initializing or finalizing.")

# --- Sidebar ---
st.sidebar.markdown("### 🧭 System Status")
st.sidebar.write(f"Current Slide ID: {slide_index}")
st.sidebar.write(f"Human Input Ready: {human_ready}")
st.sidebar.markdown("---")
st.sidebar.caption("Auto-refresh every 2s to stay in sync with robot_state.json.")
