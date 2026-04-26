#include <iostream>

#include "assignment_1_rt2_interfaces/msg/user_msg.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UserInterface : public rclcpp::Node // inherits all methods of Node class
{
    public:
        using UserMsg = assignment_1_rt2_interfaces::msg::UserMsg;

        UserInterface(): Node("user_interface"){
            publisher_ = this->create_publisher<UserMsg>("user_msg", 10); // creates publisher on topic 'topic'
            // bind creates a callable obj(function) -> where msg is created and published
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&UserInterface::get_goal_info_from_user, this)); 
        }
    private:
        void get_goal_info_from_user(){
            timer_->cancel(); // cancel the timer to stop it from calling this function again while we are waiting for user input   
            std::string input;
            while (rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(),"Enter a goal position for the robot (x y theta), 'c' (during execution) to clear the goal, or 'q' to quit: ");
                std::getline(std::cin, input);
                if (!rclcpp::ok()) {
                    break;
                }

                if (input == "q") {
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    message.msg = "q";
                    publisher_->publish(message);
                    rclcpp::shutdown();
                }
                else if (input == "c") {
                    RCLCPP_INFO(this->get_logger(), "Cancel request sent");
                    message.msg = "c";
                    publisher_->publish(message);
                }
                else {
                    std::istringstream iss(input);
                    if (!(iss >> message.x_pos >> message.y_pos >> message.theta) ) {
                        RCLCPP_ERROR(this->get_logger(), "Invalid input, please enter valid numbers for x, y and theta, 'c' (during execution) to clear the goal, or 'q' to quit");
                        return get_goal_info_from_user();
                    }
                    message.msg = "g";
                    publisher_->publish(message);
                }
            }
        }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<UserMsg>::SharedPtr publisher_;
    UserMsg message;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // initialise ros node (talker)
    rclcpp::spin(std::make_shared<UserInterface>()); // smilira to spin_some but it keeps the node executing until ctrl+C (keeps running indefinitly),
    // make_shared to link a shared pointer(auto mem management -> node is deleted when none refernces it) from the node to the obj class
    rclcpp::shutdown();
    return 0;
}