#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <iostream>
#include <thread>

#include "assignment_1_rt2_interfaces/action/navigation.hpp"
#include "assignment_1_rt2_interfaces/msg/user_msg.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include "nav_msgs/msg/odometry.hpp"

namespace assignment_1_rt2{
    struct GoalInfo {
        float goal_x;
        float goal_y;
        float goal_theta;
    };
    class NavigationActionClient : public rclcpp::Node
    {
        public:
            using Navigation = assignment_1_rt2_interfaces::action::Navigation;
            using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;
            using UserMsg = assignment_1_rt2_interfaces::msg::UserMsg;

            explicit NavigationActionClient(const rclcpp::NodeOptions & options): Node("navigation_action_client", options){
                using namespace std::placeholders;
                this->client_ptr_ = rclcpp_action::create_client<Navigation>( this, "navigation");

                publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("goal_frame", 10);
                subscription_ = this->create_subscription<UserMsg>("user_msg", 10, std::bind(&NavigationActionClient::handle_user_msg, this, _1));
                //input_thread_ = std::thread(&NavigationActionClient::get_goal_info_from_user, this);
            }

            void handle_user_msg(const UserMsg::SharedPtr msg){
                RCLCPP_INFO(this->get_logger(), "Received user message: %s", msg->msg.c_str());
                if (msg->msg == "q") {
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    if (this->goal_handle_) {
                        request_cancel_goal();
                    }
                    rclcpp::shutdown();
                }
                else if (msg->msg == "c") {
                    if (this->goal_handle_) {
                        request_cancel_goal();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "No active goal to cancel.");
                    }
                }
                else if (msg->msg == "g") {
                    if (this->goal_handle_) {
                        request_cancel_goal();
                    }
                    GoalInfo goal_info;
                    goal_info.goal_x = msg->x_pos;
                    goal_info.goal_y = msg->y_pos;
                    goal_info.goal_theta = msg->theta;
                    send_goal(goal_info);
                }
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

                nav_msgs::msg::Odometry goal_pose_msg;

                q.setRPY(0, 0, goal.goal_theta);
                goal_pose_msg.header.stamp = this->get_clock()->now();
                goal_pose_msg.header.frame_id = "world";
                goal_pose_msg.pose.pose.position.x = goal.goal_x;
                goal_pose_msg.pose.pose.position.y = goal.goal_y;
                goal_pose_msg.pose.pose.position.z = 0.0;
                goal_pose_msg.pose.pose.orientation.x = q.x();
                goal_pose_msg.pose.pose.orientation.y = q.y();
                goal_pose_msg.pose.pose.orientation.z = q.z();
                goal_pose_msg.pose.pose.orientation.w = q.w();
                publisher_->publish(goal_pose_msg);


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
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
            rclcpp::Subscription<UserMsg>::SharedPtr subscription_;
            GoalHandleNavigation::SharedPtr goal_handle_;
            //std::thread input_thread_;

            void request_cancel_goal(){
                RCLCPP_INFO(this->get_logger(), "Canceling goal");
                auto goal_handle = this->goal_handle_;
                if (!goal_handle) {
                    RCLCPP_INFO(this->get_logger(), "No active goal to cancel.");
                    return;
                }

                this->client_ptr_->async_cancel_goal(
                    goal_handle,
                    [this](std::shared_ptr<action_msgs::srv::CancelGoal::Response> cancel_result) {
                        if (!cancel_result) {
                            RCLCPP_ERROR(this->get_logger(), "Cancel response is null");
                            return;
                        }

                        if (cancel_result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
                            RCLCPP_INFO(this->get_logger(), "Goal cancel accepted");
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Goal cancel rejected (return_code=%d)", cancel_result->return_code);
                        }
                    });
            }

            void goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle){
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } 
                else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                    this->goal_handle_ = goal_handle;
                }
            }

            void feedback_callback(GoalHandleNavigation::SharedPtr, const std::shared_ptr<const Navigation::Feedback> feedback){
                RCLCPP_INFO(this->get_logger(), "Received feedback for remaining offset: x=%f, y=%f, theta=%f", 
                feedback->remaining_x, feedback->remaining_y, feedback->remaining_theta);
            }

            void result_callback(const GoalHandleNavigation::WrappedResult & result){
                this->goal_handle_ = nullptr;
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
    };  // class NavigationActionClient

}  // namespace assignment_1_rt2

RCLCPP_COMPONENTS_REGISTER_NODE(assignment_1_rt2::NavigationActionClient)