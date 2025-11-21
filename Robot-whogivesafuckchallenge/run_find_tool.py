from find_tool import init_camera, update_stream, detect_once, release_camera
import time

def main():
    init_camera(index=0)  # choose your wrist cam index

    try:
        while True:
            # keep grabbing frames
            update_stream()

            # simulate some other work here
            # time.sleep(0.01)  # optional

            cmd = input("Press d for detection, q to quit: ")

            if cmd == "d":
                result = detect_once()  # saves to last_detection.png
                print("Detection result:", result)
                print("Image saved as last_detection.png")
            elif cmd == "q":
                break

    finally:
        release_camera()

if __name__ == "__main__":
    main()
