# Modular Extensible Cobot for Human-Centered Operations (MECHO)

This repository contains files for the development of modular robotic arm platform.

Demonstration of the execution of pick and place operation is shown in the following video: [https://youtu.be/xAtnlniCpGE](https://youtu.be/xAtnlniCpGE).

[![The Pack Bottle Task Executed by the Education Cobot](https://img.youtube.com/vi/xAtnlniCpGE/0.jpg)](https://youtu.be/xAtnlniCpGE)

The platform is based on the well-known Feetech STS3215-12V smart servo motors: [https://www.dfrobot.com/product-2962.html](https://www.dfrobot.com/product-2962.html).

It was also tested with Herkulex DRS-0101, but they have requirement for 7.5V power supply and encoder with a lower resolution. Also, the Dynamixel AX-12A was considered, but the dimensions were not suitable for the application. The different motors are shown in the following image:

![Servo Motors](./Docs/images/1.jpg)

Different constructions can be built by using the following building blocks:

- 15 mm spacer (1);
- 90-degree bracket cover (2);
- 90-degree bracket (3);
- Actuator housing motor side (4);
- Actuator output (5);
- Base robot bracket (6);
- 6 mm bearing balls (7);
- Base (8).

The building blocks are shown in the following image:

![Building blocks](./Docs/images/2.jpg)

During assembly, the servomotor is attached to the motor input housing (4) of the actuator, then the actuator output part (5) is placed on top. Those two parts are held together by 6mm bearing balls (7). They are inserted through the technical holes. This ensures the attachment of the input and output parts of the actuator to each other and reduces the stress on the motor internals. It is recommended to grease the ball channel for smoother movement and to reduce wear of the parts. Thanks to the balls, the robot links are also supported by them, and not only by the servo motors. Then the output part of the actuator (5) is attached to the servo horn with bolts. The robot units are built from the remaining parts (components (1), (2) and (3)) and then attached to the input or output part of the assembled actuators with M3 bolts. M3 nuts are built into the actuator parts to prevent multiple assembly and disassembly of the robot. Typically, 3D printed parts use specialized threaded inserts, but they require a specialized assembly tool, which is not compatible with the goal of a cost-effective and widely available platform, so standard M3 nuts were chosen to be used. In addition to these parts and parts for the base (6) and (8), also a robotic gripper must be printed.

Configurations with 4, 5 and 6 degrees of freedom in a CAD environment are shown in the next figure. The actuators are shown in gray color. The links are shown in blue color.

![4, 5, 6 DOF configurations](./Docs/images/3.jpg)

Different configurations with 6 DOF in a CAD environment are shown in the next figure:

![6 DOF configurations](./Docs/images/4.jpg)

The next image shows an assembled robot with 6 degrees of freedom and its kinematics.

![6 DOF robot](./Docs/images/5.jpg)

For the robotic gripper the Micro Servo Parallel Gripper by Techniccontroller is used: [https://grabcad.com/library/micro-servo-parallel-gripper-1](https://grabcad.com/library/micro-servo-parallel-gripper-1).

The next image shows the control box of the robot. For the control of the robot an [Arduino Mega 2560 board (A)](https://store.arduino.cc/products/arduino-mega-2560-rev3) is used. for the communication with the servo motors a dedicated [Serial Bus Servo Driver Board (B)](https://www.dfrobot.com/product-3002.html) is connected to a UART port of the Arduino. An additional [25W DC-DC Power Module (C)](https://www.dfrobot.com/product-752.html) is added to the control box for providing stable 5V output for the end effector and if needed also for the Arduino.

![Control box](./Docs/images/6.jpg)

The next image shows the successfully pressed button by the robot. This experiment was intended to show that the robot is capable of interacting with real-world objects.

![Button press](./Docs/images/8.jpg)

The next image shows the successful execution of the circuit probing task of the Task Board v2023: [https://github.com/peterso/robotlearningblock](https://github.com/peterso/robotlearningblock). This experiment was intended to show that the robot has a successful repeatability of its movement execution in real tasks.

![Circuit probing task](./Docs/images/9.jpg)

The next image shows frames from the process of the execution of a task to pick up a bottle lying in a horizontal position and place it in another place, but this time in a vertical position. The position of picking up the bottle is determined by a computer vision system, which provides spatial coordinates. They are the input for the inverse kinematics problem. The solution of the inverse kinematics is used as input to the robot. The robot did extremely well in this task considering the design and the gear backlash of the very cost-effective hobby servo motors.

![Pick and place](./Docs/images/10.jpg)

## Videos

Video of the execution of the Task Board v2023 tasks is shown in the following video: [https://youtu.be/xAtnlniCpGE](https://youtu.be/Kb3CY3zpSdE).

[![Tests with Task Board v2023](https://img.youtube.com/vi/Kb3CY3zpSdE/0.jpg)](https://youtu.be/Kb3CY3zpSdE)

Demonstration of the execution of the pack bottle operation is shown in the following video: [https://youtu.be/xAtnlniCpGE](https://youtu.be/xAtnlniCpGE).

[![The Pack Bottle Task Executed by the Education Cobot](https://img.youtube.com/vi/xAtnlniCpGE/0.jpg)](https://youtu.be/xAtnlniCpGE)

## CAD files

In the folder [./CAD/](./CAD/) you can find the source CAD files. They were delevoped by using the FreeCAD software: [https://www.freecad.org/](https://www.freecad.org/).

## STL files

In the folder [./STLs](./STLs/) you can find the exported STL files which can be 3D printed.

## Kinematics

In the folder [./Kinematics](./Kinematics/) you will find a useful example Jupyter notebooks which shows how the forward and inverse kinematics can be solved by using Robotics Toolbox for Python.

![Inverse kinematics](./Docs/images/kinematics.jpg)

## Software

The folder [./Software](./Software/) contains useful Python packages and application for the control of the robot when STS3215 servo motors are used. 

You can run the control software with the following commands:

```bash
cd Software
python app.py
```

![Robot Arm Controller App](./Docs/images/controller_app.jpg)

It is still under active development.

There are also `robot_api` and `robot_arm` python packages which can be installed locally and used for coding python programs for the control of the robot. The packages can be installed locally in editable mode with: `pip install -e .`. An example program can be found in [./Software/test_api.py](./Software/test_api.py). This API is similar to the Python API for the commercial xArm cobots: [https://github.com/xArm-Developer/xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK).

You can also find the firmware for the Arduino board in the folder [./Software/arduino_controller/](./Software/arduino_controller/). This firware allows you to use a regular PWM (micro) servo for the gripper. The firmware will assign to it a virtual ID 7 and be able to control it with the same library and commands as those for the other smart STS3215-12V servos. The library for the servos is described in [https://github.com/vassar-robotics/feetech-servo-sdk](https://github.com/vassar-robotics/feetech-servo-sdk).

## License

Everything is this repository is licensed under the standard MIT license.