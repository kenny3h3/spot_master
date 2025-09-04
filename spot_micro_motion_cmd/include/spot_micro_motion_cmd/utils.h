#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace spot_micro_motion_cmd {

class Utils {
 public:
  static void publishDynamicTransform(rclcpp::Node* node, const std::string& parent_frame, const std::string& child_frame,
                                     const geometry_msgs::msg::Vector3& translation, const geometry_msgs::msg::Vector3& euler_angles) {
    static tf2_ros::TransformBroadcaster tf_broadcaster(node);
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = node->get_clock()->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;

    transform.transform.translation.x = translation.x;
    transform.transform.translation.y = translation.y;
    transform.transform.translation.z = translation.z;

    tf2::Quaternion q;
    q.setRPY(euler_angles.x, euler_angles.y, euler_angles.z);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(transform);
  }

  static void logInfo(rclcpp::Node* node, const std::string& message) {
    RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
  }
};

} // namespace spot_micro_motion_cmd
