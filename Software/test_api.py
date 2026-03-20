from robot_api.sts_robot_api import StsRobotAPI
import time
arm = StsRobotAPI(port="COM5")
arm.connect()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
arm.clean_error()
arm.set_gripper_enable(True)

gripper_open = 1000
gripper_close = 300
# arm.set_gripper_position(1000, wait=True)
# time.sleep(3)
# arm.set_gripper_position(500, wait=True)
# time.sleep(3)
arm.set_gripper_position(1000, wait=True)
time.sleep(3)

arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], wait=True, speed=50, mvacc=10)
time.sleep(3)

# posx, posy = 120, 120
# theta = 45

posx, posy = 203, 30
theta = 56

arm.set_position(posx, posy, 50, theta, 0, 180, wait=True, speed=100, mvacc=50)

# time.sleep(3)

arm.set_position(posx, posy, 10, theta, 0, 180, wait=True, speed=100, mvacc=50)

# time.sleep(3)
time.sleep(3)
arm.set_gripper_position(gripper_close, wait=True)
time.sleep(3)

arm.set_position(120, 120, 30, 45, 0, 180, wait=True, speed=100, mvacc=50)
# print(arm.get_servo_angle())
# time.sleep(3)

arm.set_servo_angle(angle=[15.6, 0, -74.86, -0.13, 76.26, 60.62],
                    wait=True, speed=100, mvacc=50)
# time.sleep(1)

arm.set_servo_angle(angle=[75, 0, -74.86, -0.13, 76.26, 60.62],
                    wait=True, speed=100, mvacc=50)
# time.sleep(1)

arm.set_servo_angle(angle=[75, 0, -70, 0, 0, 0],
                    wait=True, speed=100, mvacc=50)
# time.sleep(1)

arm.set_servo_angle(angle=[75, 25, -70, 0, 0, 0], wait=True,
                    speed=100, mvacc=50)
# time.sleep(1)

# arm.set_position(-25, 205, 110, 0, 90, 180, wait=True, speed=100, mvacc=50)
# time.sleep(3)
arm.set_servo_angle(angle=[75, 25, -70, 0, 0, 0], wait=True,
                    speed=100, mvacc=50)

# time.sleep(1)

arm.set_servo_angle(angle=[75, 25, -70, 0, -65, 0], wait=True,
                    speed=100, mvacc=50)

# time.sleep(1)
arm.set_servo_angle(angle=[52, 52, -87, -63, -70, 36], wait=True,
                    speed=100, mvacc=50)
# 71, 50, -100, 0, -67, 0
time.sleep(3)
arm.set_gripper_position(1000, wait=True)
time.sleep(5)


arm.set_servo_angle(angle=[52.7, 48.8, -88.5, -62.8, -68.4, 37.5, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[53.0, 45.4, -90.0, -63.2, -67.3, 39.2, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[53.5, 41.6, -91.3, -63.8, -65.9, 41.5, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[54.1, 37.8, -92.4, -64.4, -64.3, 43.9, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[54.7, 34.1, -93.3, -65.1, -62.7, 46.5, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[55.5, 30.1, -94.1, -66.0, -60.8, 49.4, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[56.4, 26.5, -94.7, -67.0, -58.9, 52.3, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[57.5, 22.3, -95.1, -68.4, -56.5, 56.0, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[58.7, 18.8, -95.3, -69.8, -54.3, 59.4, -32.4], wait=True,
                    speed=100, mvacc=50)


arm.set_servo_angle(angle=[60.3, 14.8, -95.3, -71.6, -51.6, 63.4, -32.4], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[62.0, 11.3, -95.0, -73.6, -49.0, 67.4, -32.4], wait=True,
                    speed=100, mvacc=50)


arm.set_servo_angle(angle=[71, 0, -50, 0, -75, 0], wait=True,
                    speed=100, mvacc=50)


arm.set_servo_angle(angle=[0, 0, 0, 0, 120, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[0, 75, 0, 0, 120, 72], wait=True,
                    speed=100, mvacc=50)


# arm.set_servo_angle(angle=[71, 58, 0, 0, 120, 72], wait=True,
#                     speed=100, mvacc=50)

arm.set_servo_angle(angle=[72, 75, 10, 0, 110, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[72, 77, 10, 0, 110, 72], wait=True,
                    speed=100, mvacc=50)

time.sleep(3)
arm.set_gripper_position(gripper_close, wait=True)
time.sleep(3)

arm.set_servo_angle(angle=[72, 60, 10, 0, 110, 72], wait=True,
                    speed=100, mvacc=50)


arm.set_servo_angle(angle=[72, 60, 10, 0, -62, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[72, 70, 10, 0, -62, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[30, 70, 10, 0, -62, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[-30, 70, 10, 0, -62, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[30, 70, 10, 0, -62, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[90, 70, 10, 0, -62, 72], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[90, 70, 10, 0, -62, -179], wait=True,
                    speed=100, mvacc=50)

arm.set_servo_angle(angle=[90, 70, 10, 0, -62, 179], wait=True,
                    speed=100, mvacc=50)

time.sleep(5)
arm.set_gripper_position(1000, wait=True)
time.sleep(3)

arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], wait=True, speed=50, mvacc=10)
time.sleep(3)
# # next 2 are OK
# arm.set_position(-25, 205, 110, 0, 0, 180, wait=True, speed=100, mvacc=50)
# # time.sleep(1)
# arm.set_position(-25, 205, 70, 0, 0, 180, wait=True, speed=100, mvacc=50)
# # time.sleep(1)
# arm.set_position(-25, 205, 60, 0, 0, 180, wait=True, speed=100, mvacc=50)
# # arm.set_position(200, 120, 100, 0, 0, 180, wait=True, speed=100, mvacc=50)

# time.sleep(3)

# arm.set_position(150, 50, 100, 0, 90, 180, wait=True, speed=100, mvacc=50)

# time.sleep(3)

# code, pos = arm.get_position()  # [x, y, z, roll, pitch, yaw]
# print(f"Position: {pos}")

# arm.set_servo_angle(angles=[0, 0, 0, 0, 0, 0], wait=True, speed=100, mvacc=50)
# code, angles = arm.get_servo_angle()
# print(f"Joint angles: {angles}")

# arm.set_gripper_position(100, wait=True)

# for i in range(90, 10, -5):
#     arm.set_position(120, 120, i, 0, 180, 0, wait=True, speed=100, mvacc=50)

# arm.set_servo_angle(angles=[0, 0, 0, 0, 0, 0], wait=True, speed=100, mvacc=50)
