#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace assignment1_rt2{
    class FramePublisher : public rclcpp::Node
    {
    public:
        explicit FramePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("frame_publisher", options)
        {
            // Declare and acquire `topic_name` parameter
            topic_name_ = this->declare_parameter<std::string>("topic_name", "odom");

            // Initialize the transform broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Subscribe to a topic to know the postion of the robot or of the goal and call handle_pose
            // callback function on each message
            std::ostringstream stream;
            stream << "/" << topic_name_.c_str();
            std::string topic = stream.str();

            auto handle_pose = [this](const std::shared_ptr<const nav_msgs::msg::Odometry> msg){
                geometry_msgs::msg::TransformStamped t;

                // Read message content and assign it to
                // corresponding tf variables
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "world";
                t.child_frame_id = topic_name_.c_str();

                // Robot can move only on a plane (2D), thus we get x and y translation
                // coordinates from the message (from the robot or the goal) and set the z coordinate to 0
                t.transform.translation.x = msg->pose.pose.position.x;
                t.transform.translation.y = msg->pose.pose.position.y;
                t.transform.translation.z = 0.0;

                // For the same reason, only rotations around z axis are possible
                // and this why we set rotation in x and y to 0 and obtain
                // rotation in z axis from the message
                t.transform.rotation.x = msg->pose.pose.orientation.x;
                t.transform.rotation.y = msg->pose.pose.orientation.y;
                t.transform.rotation.z = msg->pose.pose.orientation.z;
                t.transform.rotation.w = msg->pose.pose.orientation.w;

                // Send the transformation
                tf_broadcaster_->sendTransform(t);
            };

            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(topic, 10, handle_pose);
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string topic_name_;
    };  // class FramePublisher

}// namespace assignment1_rt2

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_rt2::FramePublisher)
