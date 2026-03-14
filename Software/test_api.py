from robot_api.sts_robot_api import StsRobotAPI

arm = StsRobotAPI(port="COM6")
arm.connect()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
arm.clean_error()
arm.set_gripper_enable(True)

arm.set_gripper_position(200, wait=True)
arm.set_position(120, 120, 40, 0, 180, 0, wait=True, speed=100, mvacc=50)

code, pos = arm.get_position()  # [x, y, z, roll, pitch, yaw]
print(f"Position: {pos}")

arm.set_servo_angle(angles=[0, 0, 0, 0, 0, 0], wait=True, speed=100, mvacc=50)
code, angles = arm.get_servo_angle()
print(f"Joint angles: {angles}")

arm.set_gripper_position(100, wait=True)

for i in range(90, 10, -5):
    arm.set_position(120, 120, i, 0, 180, 0, wait=True, speed=100, mvacc=50)

arm.set_servo_angle(angles=[0, 0, 0, 0, 0, 0], wait=True, speed=100, mvacc=50)
