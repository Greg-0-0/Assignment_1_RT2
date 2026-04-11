#include <iostream>
#include <chrono>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/msg/pose2_d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"



namespace assignment1_rt2{
    class UserInterface : public rclcpp::Node
    {
        public:
            using Pose2D = action_tutorials_interfaces::msg::Pose2D;

            explicit UserInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("user_interface", options){

                publisher1_ = this->create_publisher<nav_msgs::msg::Odometry>("goal_frame", 10);
                publisher2_ = this->create_publisher<Pose2D>("goal_pose", 10);
                startup_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(200),
                        [this]() {
                            startup_timer_->cancel();
                            this->get_goal_info_from_user();
                        }
                    );
            }

        private:
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher1_;
            rclcpp::Publisher<Pose2D>::SharedPtr publisher2_;
            rclcpp::TimerBase::SharedPtr startup_timer_;

            // Implementation for getting goal info from user and publishing it as a 2DPose message and 
            // as an Odometry message for the broadcaster to pick it up
            void get_goal_info_from_user(){
                Pose2D goal_info;
                RCLCPP_INFO(this->get_logger(),"Enter a goal position for the robot (x y theta) or q to quit: ");
                std::string input;
                std::getline(std::cin, input);
                if (input == "q") {
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    goal_info.x_pos = 300.0;
                    publisher2_->publish(goal_info);
                    rclcpp::shutdown();
                }
                else {
                    std::istringstream iss(input);
                    if (!(iss >> goal_info.x_pos >> goal_info.y_pos >> goal_info.theta)) {
                        RCLCPP_ERROR(this->get_logger(), "Invalid input, please enter valid numbers for x, y and theta or 'q' to quit");
                        return get_goal_info_from_user();
                    }
                }

                // Publish the goal as a 2DPose message
                publisher2_->publish(goal_info);

                // Also publish the goal frame as an Odometry message for the broadcaster of the goal_frame 
                // to pick it up and publish the corresponding transform
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.pose.pose.position.x = goal_info.x_pos;
                odom_msg.pose.pose.position.y = goal_info.y_pos;
                tf2::Quaternion q;
                q.setRPY(0, 0, goal_info.theta);
                odom_msg.pose.pose.orientation.x = q.x();
                odom_msg.pose.pose.orientation.y = q.y();
                odom_msg.pose.pose.orientation.z = q.z();
                odom_msg.pose.pose.orientation.w = q.w();
                publisher1_->publish(odom_msg);
                startup_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(200),
                        [this]() {
                            startup_timer_->cancel();
                            this->get_goal_info_from_user();
                        }
                    );
            }
     };  // class UserInterface
 
}  // namespace assignment1_rt2

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::UserInterface)