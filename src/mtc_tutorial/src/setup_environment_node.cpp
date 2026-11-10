#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class EnvironmentSetupNode : public rclcpp::Node {
public:
  EnvironmentSetupNode() : Node("setup_environment_node") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    object_spawned_ = false;

    timer_ = this->create_wall_timer(
      500ms, std::bind(&EnvironmentSetupNode::update_environment, this));
  }

private:
  void update_environment() {
    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_->lookupTransform("world", "tag_cube", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF 'tag_cube' not published");
      return;
    }

    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world"; // Rimane agganciato al tag_cube!
    object.header.stamp = this->get_clock()->now();

    // Posizione dell'oggetto basata sul tag + offset di -1.75 cm in Z
    object.pose.position.x = tf.transform.translation.x;
    object.pose.position.y = tf.transform.translation.y;
    object.pose.position.z = tf.transform.translation.z - 0.0175;
    object.pose.orientation.w = tf.transform.rotation.w;
    object.pose.orientation.x = tf.transform.rotation.x;
    object.pose.orientation.y = tf.transform.rotation.y;
    object.pose.orientation.z = tf.transform.rotation.z;
    // object.pose.orientation.w = 1.0;
    // object.pose.orientation.x = 0.0;
    // object.pose.orientation.y = 0.0;
    // object.pose.orientation.z = 0.0;


    if (!object_spawned_) {
      // PRIMO RUN: Definiamo la geometria del cubo e lo creiamo con ADD
      object.primitives.resize(1);
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      object.primitives[0].dimensions = { 0.035, 0.035, 0.035 };
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      
      if (psi_.applyCollisionObject(object)) {
        RCLCPP_INFO(this->get_logger(), "Oggetto generato con successo.");
        object_spawned_ = true;
      }
    } else {
      // RUN SUCCESSIVI: Diciamo a MoveIt di ri-allineare la posa basandosi sulla TF aggiornata
      object.operation = moveit_msgs::msg::CollisionObject::MOVE;
      psi_.applyCollisionObject(object);
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool object_spawned_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvironmentSetupNode>());
  rclcpp::shutdown();
  return 0;
}