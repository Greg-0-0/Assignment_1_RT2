#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#include "action_tutorials_interfaces/action/navigation2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

namespace assignment1_rt2{
    class NavigationActionServer : public rclcpp::Node
    {
        public:
            using Navigation = action_tutorials_interfaces::action::Navigation2;
            using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;

            explicit NavigationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("robot_controller", options){
                using namespace std::placeholders;
                this->action_server_ = rclcpp_action::create_server<Navigation>(this,
                    "navigation",
                    std::bind(&NavigationActionServer::handle_goal, this, _1, _2),
                    std::bind(&NavigationActionServer::handle_cancel, this, _1),
                    std::bind(&NavigationActionServer::handle_accepted, this, _1));
                publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
                subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(
                    &NavigationActionServer::odom_callback, this, _1));
                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                timer_ = this->create_wall_timer(100ms, [this]() {return this->on_timer();});
            }

        private:
            float current_pos_x = 0.0;
            float current_pos_y = 0.0;
            float current_pos_theta = 0.0;
            rclcpp_action::Server<Navigation>::SharedPtr action_server_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            rclcpp::TimerBase::SharedPtr timer_{nullptr};
            geometry_msgs::msg::TransformStamped t_;

            // Function to convert goal ID to a hex string for logging purposes
            static std::string goal_id_to_hex_string(const rclcpp_action::GoalUUID & goal_id){
                std::ostringstream oss;
                oss << std::hex << std::setfill('0');
                for (size_t i = 0; i < goal_id.size(); ++i) {
                    oss << std::setw(2) << static_cast<int>(goal_id[i]);
                }
                return oss.str();
            }

            rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Navigation::Goal> goal){
                RCLCPP_INFO(this->get_logger(), "Received request with goal: x=%f, y=%f, theta=%f",
                 goal->goal_x, goal->goal_y, goal->goal_theta );
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleNavigation> goal_handle){
                const auto goal_id = goal_id_to_hex_string(goal_handle->get_goal_id());
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal with id %s", goal_id.c_str());
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle){
                using namespace std::placeholders;
                // this needs to return quickly to avoid blocking the executor, so spin up a new thread
                std::thread{std::bind(&NavigationActionServer::execute, this, _1), goal_handle}.detach();
            }
            
            void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle){

                const auto goal_id = goal_id_to_hex_string(goal_handle->get_goal_id());
                const auto goal = goal_handle->get_goal();
                RCLCPP_INFO(this->get_logger(), "Executing goal with id %s: x=%f, y=%f, theta=%f", 
                goal_id.c_str(), goal->goal_x, goal->goal_y, goal->goal_theta);
                rclcpp::Rate loop_rate(10);
                auto feedback = std::make_shared<Navigation::Feedback>();
                auto result = std::make_shared<Navigation::Result>();
                geometry_msgs::msg::Twist msg;

                //Stage 1: Rotate to face the goal position
                RCLCPP_INFO(this->get_logger(), "Stage 1: Rotating to face the goal.");
                double desired_yaw_to_goal = atan2(goal->goal_y - current_pos_y, goal->goal_x - current_pos_x);
                double angle_diff = desired_yaw_to_goal - current_pos_theta;
                // Normalize the angle to the range [-PI, PI] (using constant M_PI)
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                while (std::abs(angle_diff) > 0.1) {
                    if (goal_handle->is_canceling()) {
                        msg.linear.x = 0.0;
                        msg.angular.z = 0.0;
                        publisher_->publish(msg);
                        result->delta_x = std::abs(goal->goal_x - current_pos_x);
                        result->delta_y = std::abs(goal->goal_y - current_pos_y);
                        result->delta_theta = std::abs(goal->goal_theta - current_pos_theta);
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal %s canceled", goal_id.c_str());
                        return;
                    }
                    // Set a proportional speed
                    msg.angular.z = (angle_diff > 0) ? 0.5 : -0.5;
                    publisher_->publish(msg);

                    // Recalculate angle difference in the loop
                    desired_yaw_to_goal = atan2(goal->goal_y - current_pos_y, goal->goal_x - current_pos_x);
                    angle_diff = desired_yaw_to_goal - current_pos_theta;
                    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                    loop_rate.sleep();
                }

                msg.angular.z = 0.0;
                publisher_->publish(msg);

                // Stage 2: Move towards the goal
                RCLCPP_INFO(this->get_logger(), "Stage 2: Moving towards the goal.");
                while(std::abs(goal->goal_x - current_pos_x) > 0.2 ||
                    std::abs(goal->goal_y - current_pos_y) > 0.2){

                    // Check if there is a cancel request
                    if (goal_handle->is_canceling()) {
                        msg.linear.x = 0.0;
                        msg.angular.z = 0.0;
                        publisher_->publish(msg);
                        result->delta_x = std::abs(goal->goal_x - current_pos_x);
                        result->delta_y = std::abs(goal->goal_y - current_pos_y);
                        result->delta_theta = std::abs(goal->goal_theta - current_pos_theta);
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal %s canceled", goal_id.c_str());
                        return;
                    }

                    // Set a speed proportional to the distance from the goal
                    static const double scaleForwardSpeed = 0.5;
                    msg.linear.x = scaleForwardSpeed * sqrt(
                        pow(t_.transform.translation.x, 2) +
                        pow(t_.transform.translation.y, 2));

                    RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x=%f, angular.z=%f",
                     msg.linear.x, msg.angular.z);

                    publisher_->publish(msg);

                    // Publish feedback
                    feedback->remaining_x = std::abs(goal->goal_x - current_pos_x);
                    feedback->remaining_y = std::abs(goal->goal_y - current_pos_y);
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Publish feedback: remaining distance to goal (x= %f, y= %f)",
                     feedback->remaining_x, feedback->remaining_y);

                    loop_rate.sleep();
                }

                // Stop the robot
                msg.linear.x = 0.0;
                publisher_->publish(msg);

                // Stage 3: Rotate to the goal orientation
                RCLCPP_INFO(this->get_logger(), "Stage 3: Rotating to the goal orientation.");
                while(std::abs(goal->goal_theta - current_pos_theta) > 0.2){
                    // Check if there is a cancel request
                    if (goal_handle->is_canceling()) {
                        result->delta_x = std::abs(goal->goal_x - current_pos_x);
                        result->delta_y = std::abs(goal->goal_y - current_pos_y);
                        result->delta_theta = std::abs(goal->goal_theta - current_pos_theta);
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal %s canceled", goal_id.c_str());
                        return;
                    }

                    // Set a speed proportional to the remaining angle difference
                    static const double scaleRotationRate = 1.0;
                    msg.angular.z = scaleRotationRate * atan2(
                        t_.transform.translation.y,
                        t_.transform.translation.x);

                    publisher_->publish(msg);

                    // Publish feedback
                    feedback->remaining_theta = std::abs(goal->goal_theta - current_pos_theta);
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Publish feedback: remaining orientation error to goal = %f",
                     feedback->remaining_theta);

                    loop_rate.sleep();
                }

                msg.angular.z = 0.0;
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Goal %s reached, stopping the robot", goal_id.c_str());

                result->delta_x = std::abs(goal->goal_x - current_pos_x);
                result->delta_y = std::abs(goal->goal_y - current_pos_y);
                result->delta_theta = std::abs(goal->goal_theta - current_pos_theta);
                goal_handle->succeed(result);
            }

            // Timer callback to lookup the transform between the robot's current position and the goal position
            void on_timer(){
                // Store frame names in variables that will be used to compute transformations
                std::string fromFrameRel = "goal_frame";
                std::string toFrameRel = "odom";

                try {
                        t_ = tf_buffer_->lookupTransform(
                            toFrameRel,
                            fromFrameRel,
                            tf2::TimePoint(),
                            50ms);
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_INFO(
                            this->get_logger(), "Error translating: Could not transform %s to %s: %s",
                            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                    }
            }

            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
                current_pos_x = msg->pose.pose.position.x;
                current_pos_y = msg->pose.pose.position.y;
                tf2::Quaternion q;
                q.setX(msg->pose.pose.orientation.x);
                q.setY(msg->pose.pose.orientation.y);
                q.setZ(msg->pose.pose.orientation.z);
                q.setW(msg->pose.pose.orientation.w);
                current_pos_theta = tf2::getYaw(q);
            }
    };  // class NavigationActionServer

}  // namespace assignment1_rt2

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::NavigationActionServer)