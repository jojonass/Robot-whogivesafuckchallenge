import socket

ROBOT_IP = "192.168.1.10"
PORT = 29999

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(3)
    s.connect((ROBOT_IP, PORT))
    f = s.makefile("rwb", buffering=0)
    print(f.readline().decode(errors="ignore").strip())  # banner
    f.write(b"robotmode\n")
    print(f.readline().decode(errors="ignore").strip())
