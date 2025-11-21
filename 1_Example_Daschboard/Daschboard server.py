# run_saved_program.py
# Run an existing Polyscope program on the robot using the Dashboard Server
# Make sure your robot is in Remote Control mode (not Local)
# and that the program path is correct.
# adjust ROBOT_IP and PROGRAM_PATH as needed.

import socket
import time

ROBOT_IP = "192.168.1.10"   # change to your robot's IP
PORT = 29999                # Dashboard Server port

PROGRAM_PATH = "Robotchallenge/rc.urp"  
# e.g., if your program is Robotchallenge/mytask.urp

def send_dashboard_command(cmd):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, PORT))
        s.sendall((cmd + "\n").encode())
        response = s.recv(1024).decode().strip()
        return response

if __name__ == "__main__":
    # 1. Load the program
    resp = send_dashboard_command(f"load {PROGRAM_PATH}")
    print(f"Load response: {resp}")
    print('The program is loaded')

    # 2. Play the program
    resp = send_dashboard_command("play")
    print(f"Play response: {resp}")
    print('The program is playing')

    # Optional: Wait a bit and stop (or let the program run)
    # time.sleep(10)
    # send_dashboard_command("stop")
