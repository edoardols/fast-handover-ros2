#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;

class CameraTfBroadcaster : public rclcpp::Node {
public:
  CameraTfBroadcaster() : Node("camera_tf_broadcaster") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&CameraTfBroadcaster::broadcast_camera_tf, this));
  }

private:
  void broadcast_camera_tf() {
    geometry_msgs::msg::TransformStamped t_cam_to_raw;
    try {
      t_cam_to_raw = tf_buffer_->lookupTransform("camera_color_optical_frame", "raw_tag_base", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return;
    }

    // Given the known posizion of one of its detected tag, find the transform of the camera respect to the world frame
    tf2::Transform t_cam_to_tag;
    tf2::fromMsg(t_cam_to_raw.transform, t_cam_to_tag);

    tf2::Transform t_tag_to_cam = t_cam_to_tag.inverse();

    geometry_msgs::msg::TransformStamped t_out;
    // Temporal syncronizzation
    // t_out.header.stamp = t_cam_to_raw.header.stamp;
    t_out.header.stamp = this->get_clock()->now();
    t_out.header.frame_id = "tag_base_optical";
    t_out.child_frame_id = "camera_color_optical_frame";
    t_out.transform = tf2::toMsg(t_tag_to_cam);

    tf_broadcaster_->sendTransform(t_out);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}