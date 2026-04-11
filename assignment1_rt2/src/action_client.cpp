#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <iostream>

#include "action_tutorials_interfaces/action/navigation2.hpp"
#include "action_tutorials_interfaces/msg/pose2_d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include "nav_msgs/msg/odometry.hpp"

namespace assignment1_rt2{
    struct GoalInfo {
        float goal_x;
        float goal_y;
        float goal_theta;
    };
    class NavigationActionClient : public rclcpp::Node
    {
        public:
            using Navigation = action_tutorials_interfaces::action::Navigation2;
            using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;

            explicit NavigationActionClient(const rclcpp::NodeOptions & options): Node("client_controller", options){
                this->client_ptr_ = rclcpp_action::create_client<Navigation>( this, "navigation");

                // Subscribe to the topic where the user interface publishes the goal pose and call goal_pose_callback on each message
                subscription_ = this->create_subscription<action_tutorials_interfaces::msg::Pose2D>(
                    "goal_pose", 10, std::bind(&NavigationActionClient::goal_pose_callback, this, std::placeholders::_1));
            }

            void send_goal(GoalInfo goal){
                using namespace std::placeholders;
                tf2::Quaternion q;

                if (!this->client_ptr_->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    return;
                }

                auto goal_msg = Navigation::Goal();
                goal_msg.goal_x = goal.goal_x;
                goal_msg.goal_y = goal.goal_y;
                goal_msg.goal_theta = goal.goal_theta;

                RCLCPP_INFO(this->get_logger(), "Sending goal");

                auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
                send_goal_options.goal_response_callback =
                std::bind(&NavigationActionClient::goal_response_callback, this, _1);
                send_goal_options.feedback_callback =
                std::bind(&NavigationActionClient::feedback_callback, this, _1, _2);
                send_goal_options.result_callback =
                std::bind(&NavigationActionClient::result_callback, this, _1);
                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            }

        private:
            rclcpp_action::Client<Navigation>::SharedPtr client_ptr_;
            rclcpp::Subscription<action_tutorials_interfaces::msg::Pose2D>::SharedPtr subscription_;

            void goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle){
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } 
                else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            }

            void feedback_callback(GoalHandleNavigation::SharedPtr, const std::shared_ptr<const Navigation::Feedback> feedback){
                RCLCPP_INFO(this->get_logger(), "Received feedback for remaining offset: x=%f, y=%f, theta=%f", 
                feedback->remaining_x, feedback->remaining_y, feedback->remaining_theta);
            }

            void result_callback(const GoalHandleNavigation::WrappedResult & result){
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        return;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                        return;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        return;
                }

                RCLCPP_INFO(this->get_logger(),"Result: x=%f, y=%f, theta=%f",
                 result.result->delta_x, result.result->delta_y, result.result->delta_theta);
            }

            void goal_pose_callback(const action_tutorials_interfaces::msg::Pose2D::SharedPtr msg){
                GoalInfo goal_info;
                goal_info.goal_x = msg->x_pos;
                goal_info.goal_y = msg->y_pos;
                goal_info.goal_theta = msg->theta;
                if (goal_info.goal_x == 300.0) {
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    rclcpp::shutdown();
                }
                send_goal(goal_info);
            }

    };  // class NavigationActionClient

}  // namespace assignment1_rt2

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::NavigationActionClient)