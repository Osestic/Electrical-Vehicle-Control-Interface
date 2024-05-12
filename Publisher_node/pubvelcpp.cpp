// Purpose: This program is a ROS2 node that listens for input from a controller and publishes velocity commands to the turtle1/cmd_vel topic.
// Created: 2024-04-07 15:00:00
// Made by Oriekaose Agholor, Andrew Pries, Brandon Smith & Edrees Ahmed

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <chrono>
#include <functional>
#include <memory>
#include <cstring>

using namespace std;
using namespace std::chrono_literals;

class PubVel : public rclcpp::Node {
public:
    PubVel() : Node("publish_velocity"), controller_fd(-1), last_left_trigger_value(0.0), last_right_trigger_value(0.0),trigger_right_held(false), trigger_left_held(false), joystick_held1(false), joystick_held2(false){
        // Initialize the publisher to publish to the turtle1/cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        // Start the input listening thread immediately upon node creation
        input_thread = std::thread(&PubVel::listenForControllerInput, this);
       
        // Initialize the timer to continuously apply the last trigger position
        timer_ = this->create_wall_timer(50ms, std::bind(&PubVel::applyLastTriggerPosition, this));
    }

    // Open the controller device file and return 0 if successful, 1 otherwise
    int set_functions(char *argv[]) {
        controller_fd = open(argv[1], O_RDONLY | O_NONBLOCK);
        if (controller_fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open controller device file.");
            return 1;
        }
        RCLCPP_INFO(this->get_logger(), "Controller device file opened successfully.");
        return 0;
    }
    
    // Close the controller device file
    void close_controller() {
        if (controller_fd != -1) {
            close(controller_fd);
            RCLCPP_INFO(this->get_logger(), "Controller device file closed successfully.");
        }
    }

    // Destructor to close the controller device file and stop the input listening thread
    ~PubVel() {
        // Signal the input listening thread to stop and wait for it to finish
        should_exit = true;
        if (input_thread.joinable()) {
            input_thread.join();
        }
        close_controller();
    }

private:
    // ROS2 publisher to publish velocity commands to the turtle1/cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int controller_fd;
    std::thread input_thread;
    bool should_exit;
    float last_left_trigger_value; 
    float last_right_trigger_value; 
    bool trigger_right_held; 
    bool trigger_left_held; 
    bool joystick_held1;
    bool joystick_held2;
    float lin_vel;
    float ang_vel;
    float break_val;

    // ROS2 message to publish velocity commands
    geometry_msgs::msg::Twist msg;

    // Listen for controller input events and handle them
    void listenForControllerInput() {
        while (!should_exit) {
            struct input_event ev;
            ssize_t bytes_read = read(controller_fd, &ev, sizeof(ev));
            if (bytes_read == sizeof(ev)) {
                handleEvent(ev);
            }
            // Minimal delay to avoid hogging the CPU
            std::this_thread::sleep_for(10ms);
        }
    }
    
    // Handle the input event and update the velocity command message
    void handleEvent(const struct input_event &ev) {
        if (ev.type == EV_KEY && ev.value == 0 && ev.code == 315) {
            //Triple horizontal line Quits the program
            if (ev.code == 315) {
                // extraction_flag = false;
                cout << "Quitting data extraction\n";
                exit(0);
            }
        }
          // Handle the joystick input
          if (ev.code == ABS_RY) {
            if (ev.value != 0) {
                joystick_held2 = true;
                break_val = ev.value * (0.003051851);
                if (abs(break_val) < 3) {
                     break_val = 0;
                }
                cout << "Breaking: " << break_val << endl;

            } else {
                joystick_held2 = false;
                break_val = 0;
            }
            
            
        }

        // Handle the joystick input
        if (ev.code == ABS_X)
        {
           if (ev.code == 0){
                if (ev.value != 0) {
                    joystick_held1 = true;
                    ang_vel = 798 + ((ev.value + 32767) * (14799 - 798)) / (32767 + 32767);
                    cout << " Joystick axis " << ev.code << endl;
                    cout << "Linear: " << lin_vel << " Angular: " << ang_vel << endl;
                }
                else {
                    joystick_held1 = false;
                    ang_vel = 0;
                }
            }
        }
        
        // Handle the triggers
        if (ev.code == ABS_Z) {
            // Handle left trigger
            lin_vel = ev.value; // Convert to a value between 0 and 1
            last_left_trigger_value = lin_vel;
            trigger_left_held = true; // Trigger is being pressed
            trigger_right_held = false; 
            msg.linear.x = 0;
            cout << "3. Left trigger " << ev.value << endl;
            cout << "Linear: " << lin_vel << " Angular: " << ang_vel << endl;
        }
        // Handle the triggers
        if (ev.code == ABS_RZ) {
            // Handle right trigger
            lin_vel = ev.value; // Convert to a value between 0 and 1
            last_right_trigger_value = lin_vel;
            trigger_right_held = true; // Trigger is being pressed
            trigger_left_held = false;
            msg.linear.y = 0;
            cout << "4. Right trigger " << ev.value << endl;
            cout << "Linear: " << lin_vel << " Angular: " << ang_vel << endl;
        }
    }

    // Apply the last trigger position to the velocity command message
    void applyLastTriggerPosition() {
        if (trigger_right_held) {
            msg.linear.x = last_right_trigger_value;
        } 
        if (trigger_left_held) {
            msg.linear.y = last_left_trigger_value;
        }
        if (joystick_held1){
            msg.angular.z = ang_vel;
        }
        if(joystick_held2) {
            msg.angular.x = break_val;
        }
 
        publisher_->publish(msg);

    }
    
};

// Main function to create the ROS2 node and spin it
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubVel>();
    if (argc > 1) {
        if (node->set_functions(argv) != 0) {
            rclcpp::shutdown();
            return 1;
        }
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}