# Xbox-Woodpecker-Electrical-Vehicle-Interface </h2>
![Cyberdirector](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/assets/42704298/2618f998-ee56-48d8-8924-cf8c0920eed5)

Oriekaose Agholor and Andrew Pries presenting this project to Senator Gary Peters and National Cyber Director, Harry Coker, Jr., during their visit to the University of Detroit Mercy

## Description
Our project was oriented around using an Xbox controller to control the Woodpecker Electric Vehicle (EV) vehicle through a combination of RS232 USB serial communication, ROS2 publisher/subscriber nodes and CAN. 

It pushed our minds and abilities to be able to create our own interfacing and electric vehicle control software that we can go on to improve up on and implement either in our own projects, the company we work for or our own company if we choose to found one. It is also a pathway into electrical vehicle cybersecurity as cars around 3 years back were being hacked through their CAN, compromising them.

The project involved installing ROS2 on two separate Raspberry Pi’s, creating a publisher C file on one Raspberry Pi, and a subscriber C file on the other.  We then were able to connect the Xbox controller and receive data into the publisher using a format for event-driven architecture. We then manipulated the data in the way we wanted to send it to the subscriber. 

Next, we were able to connect the car’s CAN bus to the CAN (Controller Area Network) shield on the subscriber Raspberry Pi. After that, we took the demo CAN send code provided and incorporated it into our ROS2 subscriber code that was receiving data from the topic and used it to send that data to the CAN of the vehicle. 

Overall, the Xbox controller can forward steer, reverse steer, brake and accelerate/decelerate the vehicle by sending data to the modules responsible for each action. We used one joystick for steering and the other for braking. Also, we used each trigger for the forward and reversing respectively. We were able to get the steering and braking to work correctly, and although the code was written for acceleration, the state of the vehicle prevented us from testing it.


## Set up
1.	Install the CAN system on two Raspberry Pis using the instructions in ```RS485-CAN-HAT-user-manuakl-en.pdf```.
2.	Connect an RS485-CAN-HAT to each of the Raspberry Pis.
3.	Integrate one of the Raspberry Pis with the Woodpecker Electric Vehicle (EV) platform by connecting the CAN High and CAN Low of the RS485-CAN-HAT to that of the EV platform.
4.	Connect the pins to be configured for parallel communication to pins on the Arbotix -M 
Arduino.

### Note:
1. Ensure both Raspberry Pi's are on the same Ethernet or internal WI-FI networks for the following steps.

## Programmed With

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![ROS 2 Logo](https://avatars.githubusercontent.com/u/3979232?s=200&v=4)


## Installation
There are 2 different folders containing the two major pieces of the project:
- [Microcontroller_Hand](https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/tree/main/Microcontroller_Hand)/

- [Robocontroller_Arm](https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/tree/main/Robocontroller_Arm)/


As the names imply, the former folder contains the code that programs the microcontroller and the UART that sends commands to the robocontroller using C, and the latter contains the code that programs the robocontroller using C++ and python. 

Steps to install the programs:
1. Download the project in [Microcontroller_Hand](https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/tree/main/Microcontroller_Hand)/ and run it on Code Composer studio to install it on a Tiva C Series TM4C123G microcontroller.
2. Download the python project in the [Arm_Position_Capture](https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/tree/main/Robocontroller_Arm/Arm_Position_Capture)/ folder contained in [Robocontroller_Arm](https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/tree/main/Robocontroller_Arm)/ and run it on a Integrated Development Environment (IDE) or text editor that can support Python 2 such as Pycharm.
3. You may need to install a virtual interpreter for Python 2.
4. Run ```PyPose.py``` from the folder which will bring up the Arm position software.
5. Run ```pypose.ino``` on the robocontroller with Arduino 1.8.5 (Do so for all arduino files). This will work with the python software to capture the position of the robot arm by just moving it and pressing the capture button.
6. Run ```main.ino```, and do so with ```poses.h``` being in its same location. This contains the code that interfaces the robocontroller with the microcontroller. You can create more gestures and include the robot commands in it.

## Usage
To run the current capabilities of the WidowX_Arm_HIWONDER_Bionic_Hand-Interface, do the following:
1. Run the ```main.c``` of the project in Code Composer studio to install it on the microcontroller.
2. Run the ```main.ino```.
3. Type in a character that enacts a specific functionality as specificied in the ```main.c``` mentioned.

The following are pictures of the  WidowX_Arm_HIWONDER_Bionic_Hand-Interface in performing different hand positions. 


- Playing the piano:
<img src="https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/assets/42704298/cb93812d-9d59-4cbb-93cf-d6da717aa819">



- Playing rock, paper and scissors:
<img src="https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/assets/42704298/20376c5a-8b07-44d5-b52b-3d32332adce1">

[Video Demo](https://youtu.be/vg_HYsMw0Hg) 

To capture new positions and sequences for the WidowX_Arm_HIWONDER_Bionic_Hand-Interface, do the following:
1. Run the ```PyPose.py```.
2. Run the ```pypose.ino```.
3. Select the maximum resolution, 4096.
4. Select a serial port from the config tab.
5. Select the pose editor from the tools and create a new position.
6. Select capture position.
7. Set can be used to set the position.
8. Select the tools tab and select sequence editor.
9. Load in the positions you have captured and choose the duration you wish for each one.
10. Export the positions and sequences you have created by clicking the export to AVR fromt the tools tab.
11. You can save the application as a file you can always load.

The image of how the pose capturing looks like is shown below:
![Screenshot 2023-12-17 200032](https://github.com/Osestic/WidowX_Arm_HIWONDER_Bionic_Hand-Interface/assets/42704298/a565100c-5b07-490d-9a5a-44bf54016e5d)

 
## Authors
This project was a result of the collaborative effort of:
1. Oriekaose Chukwuyem Agholor - Aspiring Robotics and Mechatronic Systems Engineer [https://github.com/Osestic]
2. Evan Varga - Aspiring Electrical Engineer [https://github.com/EvanVarga]

## License
This project is licensed under the “Commons Clause” License Condition v1.0. See ```LICENSE``` for more information.


## How to Contribute
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.1-4baaaa.svg)](code_of_conduct.md)


## Recommendations
- In the ```main.c``` , spend less time in the interrupts
- Include your own gestures and actions
- Allow other persons to view the project
- Act on their feedback accordingly

