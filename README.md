# Modular Extensible Cobot for Human-Centered Operations (MECHO)

This repository contains files for the development of modular robotic platform.

The platform is based on the well-known Feetech STS3215 smart servo motors.

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

![Building blocks](./Docs/Fig.%202.jpg)

During assembly, the servomotor is attached to the motor input housing (4) of the actuator, then the actuator output part (5) is placed on top. Those two parts are held together by 6mm bearing balls (7). They are inserted through the technical holes. This ensures the attachment of the input and output parts of the actuator to each other and reduces the stress on the motor internals. It is recommended to grease the ball channel for smoother movement and to reduce wear of the parts. Thanks to the balls, the robot links are also supported by them, and not only by the servo motors. Then the output part of the actuator (5) is attached to the servo horn with bolts. The robot units are built from the remaining parts (components (1), (2) and (3)) and then attached to the input or output part of the assembled actuators with M3 bolts. M3 nuts are built into the actuator parts to prevent multiple assembly and disassembly of the robot. Typically, 3D printed parts use specialized threaded inserts, but they require a specialized assembly tool, which is not compatible with the goal of a cost-effective and widely available platform, so standard M3 nuts were chosen to be used. In addition to these parts and parts for the base (6) and (8), also a robotic gripper must be printed.

Configurations with 4, 5 and 6 degrees of freedom in a CAD environment are shown in the next figure. The actuators are shown in gray color. The links are shown in blue color.

![4, 5, 6 DOF configurations](./Docs/Fig.%203.jpg)

Different configurations with 6 DOF in a CAD environment are shown in the next figure:

![6 DOF configurations](./Docs/Fig.%204.jpg)

The next image shows an assembled robot with 6 degrees of freedom and its kinematics.

![6 DOF robot](./Docs/Fig.%205.jpg)

The next image shows the control box of the robot. For the control of the robot an Arduino Mega 2560 (A) is used. for the communication with the servo motors a dedicated Serial Bus Servo Driver Board (B) is connected to a UART port of the Arduino. An additional 25W DC-DC Power Module (C) is added to the control box for providing stable 5V output for the end effector and if needed also for the Arduino.

![Control box](./Docs/Fig.%206.jpg)

The next image shows the successfully pressed button by the robot. This experiment was intended to show that the robot is capable of interacting with real-world objects.

![Button press](./Docs/Fig.%208.jpg)

The next image shows the successful execution of the circuit probing task of the Task Board v2023: [https://github.com/peterso/robotlearningblock](https://github.com/peterso/robotlearningblock). This experiment was intended to show that the robot has a successful repeatability of its movement execution in real tasks.

![Circuit probing task](./Docs/Fig.%209.jpg)

The next image shows frames from the process of the execution of a task to pick up a bottle lying in a horizontal position and place it in another place, but this time in a vertical position. The position of picking up the bottle is determined by a computer vision system, which provides spatial coordinates. They are the input for the inverse kinematics problem. The solution of the inverse kinematics is used as input to the robot. The robot did extremely well in this task considering the design and the gear backlash of the very cost-effective hobby servo motors.

![Pick and place](./Docs/Fig.%2010.jpg)

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

## Software

The folder [./Software](./Software/) contains useful Python packages and application for the control of the robot when STS3215 servo motors are used. This is still under active development.

## License

Everything is this repository is licensed under the standard MIT license.