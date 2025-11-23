import URBasic
import time

import robotiq_gripper

host = "192.168.0.9"  # E.g. a Universal Robot offl
# ine simulator, please adjust to match your IP
acc = 0.1  # 0.9
vel = 0.1  # 0.9

def ExampleurScript():
    robotModle = URBasic.robotModel.RobotModel()

    robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModle)
    robot.reset_error()

    robot.movel(pose=[-0.3366, -0.2431, 0.15, -1.8471, -2.5203, 0.0339], a=0.3, v=vel)
    time.sleep(1)
    robot.end_force_mode()
    robot.close()


if __name__ == '__main__':
    try:
        ExampleurScript()
    except Exception as e:
        print(f"An error occurred: {e}")