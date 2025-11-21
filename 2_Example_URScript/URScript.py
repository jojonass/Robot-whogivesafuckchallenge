# ur_robotiq_pick_place.py
# Requires: Robotiq URCap installed on the UR controller (so rq_* script funcs exist)
# Action sequence:
#   Activate gripper -> open -> movej home -> move to pick (movej + movel) -> close
#   -> movep via -> movej pre-place -> movel place -> open -> movej home

import socket
import sys

ROBOT_IP = "192.168.1.10"   # <-- set your robot's IP
PORT = 30001               # Primary client (30001) or Secondary (30002)


UR_SCRIPT = """
def test_move():
  movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.50)
end
test_move()
"""

UR_SCRIPT = """
def pick_place_demo():
  a_joint = 1.2
  v_joint = 0.25
  a_lin   = 1.0
  v_lin   = 0.15

  # Home joint move
  movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.50)
  
  home_j      = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
  pre_pick_p  = p[0.400, -0.200, 0.250, 0, 3.1416, 0]
  pick_p      = p[0.400, -0.200, 0.170, 0, 3.1416, 0]
  via_p       = p[0.4500, 0.000, 0.300, 0, 3.1416, 0]
  pre_place_p = p[0.450, 0.200, 0.250, 0, 3.1416, 0]
  place_p     = p[0.450, 0.200, 0.170, 0, 3.1416, 0]

  # --- Activate gripper ---
  textmsg("Activating Robotiq gripper")
  socket_open("127.0.0.1", 63352)
  socket_send_string("SET ACT 1\n")
  sleep(0.5)
  socket_send_string("SET GTO 1\n")
  sleep(0.5)
  socket_send_string("SET SPE 255\n")
  socket_send_string("SET FOR 150\n")

  # --- Open gripper before pick ---
  textmsg("Opening gripper")
  socket_send_string("SET POS 0\n")
  sleep(1.0)

  # --- Motion to pick ---
  textmsg("Moving to home_j")
  movej(home_j, a=a_joint, v=v_joint)
  textmsg("Moving to pre-pick position")
  movej(get_inverse_kin(pre_pick_p), a=a_joint, v=v_joint)
  textmsg("Moving to pick position")
  movel(pick_p, a=a_lin, v=v_lin)

  # --- Close gripper to grasp ---
  textmsg("Closing gripper")
  socket_send_string("SET POS 255\n")
  sleep(1.0)

  # --- Move via intermediate position ---
  textmsg("Moving via position")
  movep(via_p, a=a_lin, v=v_lin)

  # --- Move to place ---
  textmsg("Moving to pre-place position")
  movej(get_inverse_kin(pre_place_p), a=a_joint, v=v_joint)
  textmsg("Moving to place position")
  movel(place_p, a=a_lin, v=v_lin)

  # --- Open gripper to release ---
  textmsg("Opening gripper to release")
  socket_send_string("SET POS 0\n")
  sleep(1.0)

  # --- Back home ---
  textmsg("Returning to home position")
  movej(home_j, a=a_joint, v=v_joint)

  # --- Close gripper socket ---
  socket_close()
  textmsg("Pick and place complete")
end

pick_place_demo()
"""


def send_urscript(script: str, ip: str = ROBOT_IP, port: int = PORT):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5.0)
            s.connect((ip, port))
            s.sendall(script.encode("utf-8"))
        print("URScript sent.")
    except Exception as e:
        print(f"Error sending URScript: {e}", file=sys.stderr)

if __name__ == "__main__":
    send_urscript(UR_SCRIPT)
