// Final Project Subscribing to a topic and sending data to a CAN bus
// Created: 2024-04-07 15:00:00
// Made by Oriekaose Agholor, Andrew Pries, Brandon Smith & Edrees Ahmed

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <ctype.h>
#include <sstream>
#include <string>

using namespace std;
using std::placeholders::_1;

class subPose : public rclcpp::Node {
public:
    subPose() : Node("subscribe_pose") {
        // Create a subscriber to the topic "turtle1/cmd_vel"
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10, std::bind(&subPose::topic_callback, this, _1));

        // Initialize CAN socket
        system("sudo ip link set can0 type can bitrate 500000");
        system("sudo ifconfig can0 txqueuelen 65536");
        system("sudo ifconfig can0 up");
        printf("this is a can send demo\r\n");

        // Create CAN socket
        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) {
            perror("socket PF_CAN failed");
            RCLCPP_INFO(this->get_logger(),"error opening CAN socket");
            return;
        }

        // Specify can0 device
        strcpy(ifr.ifr_name, "can0");
        ret = ioctl(s, SIOCGIFINDEX, &ifr);
        if (ret < 0) {
            perror("ioctl failed");
            RCLCPP_INFO(this->get_logger(),"ioctl failed");
            return;
        }

        // Bind the socket to can0
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
        if (ret < 0) {
            perror("bind failed");
            RCLCPP_INFO(this->get_logger(),"bind failed");

            return;
        }
     
        printf("Initialized CAN socket and bound to can0\n");

        // Disable filtering rules, do not receive packets, only send
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
      
    }

    // Destructor
    ~subPose() {
        system("sudo ifconfig can0 down");
        close(s);
    }

private:
    // Subscriber to the topic "turtle1/cmd_vel" and send data to CAN bus
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    // CAN socket file descriptor and address structure for socket binding 
    int s, ret;
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    // Callback function for the subscriber
    void topic_callback(const geometry_msgs::msg::Twist &msg) const {
    RCLCPP_INFO(this->get_logger(),"Received Twist message. Sending to CAN...\n");
    
    // Disable filtering rules, do not receive packets, only send
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    
    // Set char and DLC
    frame.can_dlc = 8;
    unsigned char input[8];

    int break_value = msg.angular.x; // Break value from the publisher
    int reverse = msg.linear.x; // Reverse from the publisher
    int forward = msg.linear.y; // Forward from the publisher
    int controller_angle_value = msg.angular.z; // Controller angle value from the publisher
    int back_wheels = -msg.angular.z + 15597; // Back wheels oppisite of controller angle value

    // When forward and reverse are both 0, the car is not moving
    if (reverse < 0 && forward < 0) {
        
        frame.can_id = 0x201; // Set the CAN ID

        // Set the input values
        input[0] = 0x22;
        input[1] = 0x04;
        input[2] = 0x00;
        input[3] = 0x00;
        input[4] = 0x00;
        input[5] = 0x00;
        input[6] = 0x00;
        input[7] = 0x00;

        // Set the frame data
        frame.data[0] = input[0];
        frame.data[1] = input[1];
        frame.data[2] = input[2];
        frame.data[3] = input[3];
        frame.data[4] = input[4];
        frame.data[5] = input[5];
        frame.data[6] = input[6];
        frame.data[7] = input[7];     
        
        // Print the frame data
        cout << "\nSteering\n";
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
        int i = 0;
        for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

        // Send message
        int nbytes = write(s, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            printf("Send Error frame!\n");
            printf("number of bytes %d\n",nbytes);
        }   

    }
    
    // When reverse is greater than 0, the car is moving in reverse
    if (reverse > 0) {
        frame.can_id = 0x201;
        if (reverse > 0 && reverse < 46.5) {
            input[0] = 0x00;
            input[1] = 0x2E;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        } 
        else if (reverse > 46.5 && reverse < 93) {
            input[0] = 0x00;
            input[1] = 0x5C;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 93 && reverse < 139.5) {
            input[0] = 0x00;
            input[1] = 0x8A;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 139.5 && reverse < 186) {
            input[0] = 0x00;
            input[1] = 0xB8;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 186 && reverse < 232.5) {
            input[0] = 0x00;
            input[1] = 0xE6;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 232.5 && reverse < 279) {
            input[0] = 0x01;
            input[1] = 0x14;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 279 && reverse < 325.5) {
            input[0] = 0x01;
            input[1] = 0x42;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 325.5 && reverse < 372) {
            input[0] = 0x01;
            input[1] = 0x70;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 372 && reverse < 418.5) {
            input[0] = 0x01;
            input[1] = 0x9E;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 418.5 && reverse < 465) {
            input[0] = 0x01;
            input[1] = 0xCC;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 465 && reverse < 511.5) {
            input[0] = 0x01;
            input[1] = 0xFA;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 511.5 && reverse < 558) {
            input[0] = 0x02;
            input[1] = 0x28;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 558 && reverse < 604.5) {
            input[0] = 0x02;
            input[1] = 0x56;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 604.5 && reverse < 651) {
            input[0] = 0x02;
            input[1] = 0x84;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 651 && reverse < 697.5) {
            input[0] = 0x02;
            input[1] = 0xB2;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 697.5 && reverse < 744) {
            input[0] = 0x02;
            input[1] = 0xE0;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 744 && reverse < 790.5) {
            input[0] = 0x03;
            input[1] = 0x0E;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 790.5 && reverse < 837) {
            input[0] = 0x03;
            input[1] = 0x3C;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 837 && reverse < 883.5) {
            input[0] = 0x03;
            input[1] = 0x6A;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 883.5 && reverse < 930) {
            input[0] = 0x03;
            input[1] = 0x78;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 930 && reverse < 976.5) {
            input[0] = 0x03;
            input[1] = 0xC6;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (reverse > 976.5 && reverse < 1023) {
            input[0] = 0x03;
            input[1] = 0xF4;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else {
            input[0] = 0x04;
            input[1] = 0x22;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }

        // Set the frame data
        frame.data[0] = input[0];
        frame.data[1] = input[1];
        frame.data[2] = input[2];
        frame.data[3] = input[3];
        frame.data[4] = input[4];
        frame.data[5] = input[5];
        frame.data[6] = input[6];
        frame.data[7] = input[7];     
        
        // Print the frame data
        cout << "\nSteering\n";
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
        int i = 0;
        for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

        // Send message
        int nbytes = write(s, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            printf("Send Error frame!\n");
            printf("number of bytes %d\n",nbytes);
        } 

    
    }

    // When forward is greater than 0, the car is moving forward
    if(forward > 0) {
        frame.can_id = 0x201;
        if (forward > 0 && forward < 46.5) {
            input[0] = 0x04;
            input[1] = 0x50;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        } 
        else if (forward > 46.5 && forward < 93) {
            input[0] = 0x04;
            input[1] = 0x7E;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 93 && forward < 139.5) {
            input[0] = 0x04;
            input[1] = 0xAC;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 139.5 && forward < 186) {
            input[0] = 0x04;
            input[1] = 0xDA;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 186 && forward < 232.5) {
            input[0] = 0x05;
            input[1] = 0x08;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 232.5 && forward < 279) {
            input[0] = 0x05;
            input[1] = 0x36;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 279 && forward < 325.5) {
            input[0] = 0x05;
            input[1] = 0x64;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 325.5 && forward < 372) {
            input[0] = 0x05;
            input[1] = 0x92;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 372 && forward < 418.5) {
            input[0] = 0x05;
            input[1] = 0xC0;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 418.5 && forward < 465) {
            input[0] = 0x05;
            input[1] = 0xEE;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 465 && forward < 511.5) {
            input[0] = 0x06;
            input[1] = 0x1C;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 511.5 && forward < 558) {
            input[0] = 0x06;
            input[1] = 0x4A;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 558 && forward < 604.5) {
            input[0] = 0x06;
            input[1] = 0x78;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 604.5 && forward < 651) {
            input[0] = 0x06;
            input[1] = 0xA6;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 651 && forward < 697.5) {
            input[0] = 0x06;
            input[1] = 0xD4;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 697.5 && forward < 744) {
            input[0] = 0x07;
            input[1] = 0x02;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 744 && forward < 790.5) {
            input[0] = 0x07;
            input[1] = 0x30;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 790.5 && forward < 837) {
            input[0] = 0x07;
            input[1] = 0x5E;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 837 && forward < 883.5) {
            input[0] = 0x07;
            input[1] = 0x8C;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 883.5 && forward < 930) {
            input[0] = 0x07;
            input[1] = 0xBA;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 930 && forward < 976.5) {
            input[0] = 0x07;
            input[1] = 0xE8;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else if (forward > 976.5 && forward < 1023) {
            input[0] = 0x07;
            input[1] = 0xF8;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }
        else {
            input[0] = 0x04;
            input[1] = 0x22;
            input[2] = 0x00;
            input[3] = 0x00;
            input[4] = 0x00;
            input[5] = 0x00;
            input[6] = 0x00;
            input[7] = 0x00;
        }

        // Set the frame data
        frame.data[0] = input[0];
        frame.data[1] = input[1];
        frame.data[2] = input[2];
        frame.data[3] = input[3];
        frame.data[4] = input[4];
        frame.data[5] = input[5];
        frame.data[6] = input[6];
        frame.data[7] = input[7];     
        
        // Print the frame data
        cout << "\nSteering\n";
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
        int i = 0;
        for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

        // Send message
        int nbytes = write(s, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            printf("Send Error frame!\n");
            printf("number of bytes %d\n",nbytes);
        } 

    }
    
    // When controller angle value is greater than 798 and less than 7100 or greater than 8300 and less than 14799, the car is turning
    if ( (controller_angle_value > 798 && controller_angle_value < 7100) || (controller_angle_value > 8300 && controller_angle_value < 14799)  ) {
        frame.can_id = 0x219; // Set the CAN ID

        // Convert the controller angle value to hex
        std::stringstream hexVal;
        std::stringstream finalHexVal;
        hexVal << std::hex << controller_angle_value;
        finalHexVal << std::hex << (controller_angle_value); 
        std::string hexOutput(finalHexVal.str());
        
        // Convert the hex value to decimal 
        uint16_t finalDecVal;
        finalHexVal >> finalDecVal;

        // Extracting upper and lower bytes
        uint8_t lowerByte = finalDecVal & 0xFF;
        uint8_t upperByte = (finalDecVal >> 8) & 0xFF;
        
        // Print the lower and upper bytes
        printf("%x", lowerByte & 0xff);
        printf("%x", upperByte & 0xff);

        // Set the input values
        input[0] = 0x40;
        input[1] = 0xFA;
        input[2] = lowerByte;
        input[3] = upperByte;
        input[4] = 0x00;
        input[5] = 0x00;
        input[6] = 0x00;
        input[7] = 0x00;

        // Set the frame data
        frame.data[0] = input[0];
        frame.data[1] = input[1];
        frame.data[2] = input[2];
        frame.data[3] = input[3];
        frame.data[4] = input[4];
        frame.data[5] = input[5];
        frame.data[6] = input[6];
        frame.data[7] = input[7];     
        
        // Print the frame data
        cout << "\nSteering\n";
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
        int i = 0;
        for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

        // Send message
        int nbytes = write(s, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            printf("Send Error frame!\n");
            printf("number of bytes %d\n",nbytes);
        }   
                                                                                                                        
    }

    if ( (back_wheels > 798 && back_wheels < 7100) || (back_wheels > 8300 && back_wheels < 14799) ) {
        frame.can_id = 0x220; // Set the CAN ID

        // Convert the back wheels value to hex
        std::stringstream hexVal;
        std::stringstream finalHexVal;
        hexVal << std::hex << back_wheels;
        finalHexVal << std::hex << (back_wheels); 
        std::string hexOutput(finalHexVal.str());

     
        // Convert the hex value to decimal
        uint16_t finalDecVal;
        finalHexVal >> finalDecVal;

        // Extracting upper and lower bytes
        uint8_t lowerByte = finalDecVal & 0xFF;
        uint8_t upperByte = (finalDecVal >> 8) & 0xFF;
        
        // Print the lower and upper bytes
        printf("%x", lowerByte & 0xff);
        printf("%x", upperByte & 0xff);
       
        // Set the input values
        input[0] = 0x40;
        input[1] = 0xFA;
        input[2] = lowerByte;
        input[3] = upperByte;
        input[4] = 0x00;
        input[5] = 0x00;
        input[6] = 0x00;
        input[7] = 0x00;

        // Set the frame data
        frame.data[0] = input[0];
        frame.data[1] = input[1];
        frame.data[2] = input[2];
        frame.data[3] = input[3];
        frame.data[4] = input[4];
        frame.data[5] = input[5];
        frame.data[6] = input[6];
        frame.data[7] = input[7];     
        
        // Print the frame data
        cout << "\nSteering\n";
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
        int i = 0;
        for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

        // Send message
        int nbytes = write(s, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            printf("Send Error frame!\n");
            printf("number of bytes %d\n",nbytes);
        }   
                                                                                                                        
    }

    // When break value is greater than 0, the car is breaking
    if (break_value > 0) {
        frame.can_id = 0x154; // Set the CAN ID

        // Convert the break value to hex
        std::stringstream hexVal;
        std::stringstream finalHexVal;
        hexVal << std::hex << break_value;
        finalHexVal << std::hex << (break_value); 
        std::string hexOutput(finalHexVal.str());

        // Convert the hex value to decimal
        uint8_t finalDecVal;
        finalHexVal >> finalDecVal;
        uint8_t byte = finalDecVal & 0xFF;
        
        // Print the byte
        cout << endl;
        printf("%x", byte & 0xff);
      
        // Set the input values
        input[0] = 0x01;
        input[1] = 0x00;
        input[2] = byte;
        input[3] = 0x00;
        input[4] = 0x00;
        input[5] = 0x00;
        input[6] = 0x00;
        input[7] = 0x00;

        // Set the frame data
        frame.data[0] = input[0];
        frame.data[1] = input[1];
        frame.data[2] = input[2];
        frame.data[3] = input[3];
        frame.data[4] = input[4];
        frame.data[5] = input[5];
        frame.data[6] = input[6];
        frame.data[7] = input[7];     
        
        // Print the frame data
        cout << "\nBraking\n";
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
        int i = 0;
        for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

        // Send message
        int nbytes = write(s, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            printf("Send Error frame!\n");
            printf("number of bytes %d\n",nbytes);
        }   
    }

}

};

// Main function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<subPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}