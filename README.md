# Xbox Woodpecker Electrical Vehicle Interface </h2>
![Cyberdirector](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/assets/42704298/2618f998-ee56-48d8-8924-cf8c0920eed5)

Oriekaose Agholor and Andrew Pries presenting this project to Senator Gary Peters (on the far left) and National Cyber Director, Harry Coker, Jr. (on the right), during their visit to the University of Detroit Mercy

![image](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/assets/42704298/af71fddf-a03f-4eec-9f53-9b2c77fc29a9)

The System Block Diagram
## Description
Our project was oriented around using an Xbox controller to control the Woodpecker Electric Vehicle (EV) vehicle through a combination of RS232 USB serial communication, ROS2 publisher/subscriber nodes and CAN. 

It pushed our minds and abilities to be able to create our own interfacing and electric vehicle control software that we can go on to improve upon and implement either in our own projects, the company we work for or our own company if we choose to found one. It is also a pathway into electrical vehicle cybersecurity as cars around 3 years back were being hacked through their CAN, compromising them.

The project involved installing ROS2 on two separate Raspberry Pi’s, creating a publisher C file on one Raspberry Pi, and a subscriber C file on the other.  We then were able to connect the Xbox controller and receive data into the publisher using a format for event-driven architecture. We then manipulated the data in the way we wanted to send it to the subscriber. 

Next, we were able to connect the car’s CAN bus to the CAN (Controller Area Network) shield on the subscriber Raspberry Pi. After that, we took the demo CAN send code provided and incorporated it into our ROS2 subscriber code that was receiving data from the topic and used it to send that data to the CAN of the vehicle. 

Overall, the Xbox controller can forward steer, reverse steer, brake and accelerate/decelerate the vehicle by sending data to the modules responsible for each action. We used one joystick for steering and the other for braking. Also, we used each trigger for the forward and reversing respectively. We were able to get the steering and braking to work correctly, and although the code was written for acceleration, the state of the vehicle prevented us from testing it.


## Set up
1. Install Ubuntu and Ros2 on both Raspberry Pis.
2.	Install the CAN system on two Raspberry Pis using the instructions in ```RS485-CAN-HAT-user-manuakl-en.pdf```.
3.	Connect an RS485-CAN-HAT to each of the Raspberry Pis.
4.	Integrate one of the Raspberry Pis with the Woodpecker Electric Vehicle (EV) platform by connecting the CAN High and CAN Low of the RS485-CAN-HAT to that of the EV platform.

### Note:
  - Ensure both Raspberry Pi's are on the same Ethernet or internal WI-FI network for the following steps.
  - Make sure the ```~/.bashrc``` file of both Raspberry Pi's have the same ROS_DOMAIN_ID and that ROS_LOCALHOST_ONLY is either set to 0 or deleted. 

## Programmed With

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)(ROS2)


## Installation
There are 2 different folders containing the two major pieces of the project:
- [Publisher_node](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/tree/main/Publisher_node)

- [Subscriber_node](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/tree/main/Subscriber_node)


As the names imply, the first folder contains the code for setting up and creating the publisher node to be installed on one Raspberry Pi, and the other folder contains the code for the subscriber node to be installed on the Raspberry Pi integrated with the EV platform. 

Steps to install the program:
1. Create a ROS2 package on both Raspberry Pis by doing the following

   a. Create a new folder, your workspace, on both devices and create a ```src``` folder in both.

   b. In the Bash terminals of both Raspberry Pis run the code below.
```bash
#Replace the address/address/ with the src folder location
cd address/yourWorkSpace/src

#Creates a package
#Replace yourPackageName with any name of your choice
ros2 create pkg yourPackageName
```

3. Copy the ```subscribe.cpp``` file from the [Subscriber_node](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/tree/main/Subscriber_node) folder and paste it in yourPackageName -> src of the Raspberry Pi (Subscriber node) integrated with the EV platform.
4. Copy the ```pubvelcpp.cpp``` file from the [Publisher_node](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/tree/main/Publisher_node) folder and paste it in yourPackageName -> src of the other Raspberry Pi (Publisher node).  
5. Replace the ```CMakeLists.txt``` and ```package.xml``` files with the ones contained in the aforementioned folders for the specific nodes.
6. Run ```colcon build``` in the Bash terminal in your workspaces at the same level as the ```src``` folders you created initially.
7. Afterwards, execute ```source install/local_setup.bash``` in the ```install``` folders within the ```src``` folders.

## Usage
To use an Xbox to control the WoodpeckervElectrical Vehicle Interface, do the following:
1. Run  ls ```/dev/input/ ``` in the bash terminal of the publisher node.
2. Connect the Xbox controller to the publisher node.
3. Run  ls ```/dev/input/ ``` again.
4. Compare both outputs and  make note of the new input device file name that starts with ```event```.
5. Run ```ros2 run yourPackageName pubvelcpp /dev/input/inputDeviceFileName``` in the Bash terminal in your workspace at the same level as the ```src``` folder.
6. Then, on your subscriber node run ```ros2 run yourPackageName subscribe ``` in the Bash terminal in your workspace at the same level as the ```src``` folder.
7. Now, you can utilise the Xbox to control the EV platform using the button control layout provided below.

![image](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/assets/42704298/90b6bb2a-35ac-4027-9f35-61e07d7346bc)
The Xbox Control Layout We Specified for Controlling the Electrical Vehicle Platform</p>

### Development Process
[Check out the development process in my YouTube playlist](https://www.youtube.com/playlist?list=PL0DIBq2mP_CGUCG4vjxHVTH7tr-DX0TGv)


### Presentation and Demo to Senator Gary Peters (on the far left) and National Cyber Director, Harry Coker, Jr. 
![vehicle present](https://github.com/Osestic/Xbox-Woodpecker-Electrical-Vehicle-Interface/assets/42704298/d906fbe6-7b2e-4556-9d71-ab96f1bfa41f)
[Video Demo](https://youtu.be/h61ZqsItLts?si=CaRQgDzqr-zjd_1M) 

 
## Authors
This project was a result of the collaborative effort of:
1. [Oriekaose Chukwuyem Agholor](https://www.linkedin.com/in/oriekaose-agholor/) - Aspiring Robotics and Mechatronic Systems Engineer 
2. [Andrew Pries](https://www.linkedin.com/in/andrew-pries-08781721b/) - Aspiring Computer Scientists
3. [Brandom Smith](https://www.linkedin.com/in/brandon-smith4/) - Aspiring Robotics and Mechatronic Systems Engineer
4. [Edrees Ahmed](https://www.linkedin.com/in/edrees-ahmed-1a2186168/)- Aspiring Electrical Engineer

## License
This project is licensed under the “Commons Clause” License Condition v1.0. See ```LICENSE``` for more information.


## How to Contribute
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.1-4baaaa.svg)](code_of_conduct.md)


## Recommendations
- Utilise a more robust means of acceleration using a set of equations to convert the publisher data to correspond to hexadecimal values that are sent to the CAN bus for acceleration if possible.
- Allow other persons to view the project.
- Act on their feedback accordingly.

