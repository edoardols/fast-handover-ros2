#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode {
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

private:
  mtc::Task setupBaseTask(const std::string& task_name);
  mtc::Task createFirstTask();
  mtc::Task createSecondTask();
  mtc::Task createThirdTask();
  mtc::Task createForthTask();

  bool planAndExecute(mtc::Task& task);

  rclcpp::Node::SharedPtr node_;

  // Planners
  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

  // Hardware configuration
  const std::string arm_group_name_ = "manipulator";
  const std::string hand_group_name_ = "gripper";
  const std::string hand_frame_ = "end_effector_link";
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options) : node_{ std::make_shared<rclcpp::Node>("mtc_task_node", options) } {
  // Initialization and configuration of planners
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner_->setMaxVelocityScalingFactor(0.2);
  sampling_planner_->setMaxAccelerationScalingFactor(0.02);

  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner_->setMaxVelocityScalingFactor(0.2);
  interpolation_planner_->setMaxAccelerationScalingFactor(0.02);

  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(0.2);
  cartesian_planner_->setMaxAccelerationScalingFactor(0.02);
  cartesian_planner_->setStepSize(.01);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

void MTCTaskNode::doTask() {
  RCLCPP_INFO(LOGGER, "Avvio della sequenza dei 4 task...");

  auto task1 = createFirstTask();
  if (planAndExecute(task1)) {
    auto task2 = createSecondTask();
    if (planAndExecute(task2)) {
      auto task3 = createThirdTask();
      if (planAndExecute(task3)) {
        auto task4 = createForthTask();
        if (planAndExecute(task4)) {
          RCLCPP_INFO(LOGGER, "TUTTI I 4 I TASK SONO STATI ESEGUITI CON SUCCESSO!");
          return;
        }
      }
    }
  }

  RCLCPP_ERROR(LOGGER, "La sequenza si è interrotta a causa di un fallimento.");
}

bool MTCTaskNode::planAndExecute(mtc::Task& task) {
  try {
    task.init();
  }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Inizializzazione fallita: " << e.what());
    return false;
  }

  // 10 seconds timeout for planning
  if (!task.plan(10)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Pianificazione del task fallita");
    return false;
  }

  // Mostra la soluzione in RViz
  task.introspection().publishSolution(*task.solutions().front());

  // Esegui sul robot reale/simulato
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Esecuzione del task fallita");
    return false;
  }

  RCLCPP_INFO(LOGGER, "Task completato con successo!");
  return true;
}

mtc::Task MTCTaskNode::setupBaseTask(const std::string& task_name) {
  mtc::Task task;
  task.stages()->setName(task_name);
  task.loadRobotModel(node_);
  
  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", hand_group_name_);
  task.setProperty("ik_frame", hand_frame_);

  // Sync robot state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  return task;
}

mtc::Task MTCTaskNode::createFirstTask() {
  mtc::Task task = setupBaseTask("Prepare to grasp");
  
  auto stage_go_home = std::make_unique<mtc::stages::MoveTo>("move to home", interpolation_planner_);
  stage_go_home->setGroup(arm_group_name_);
  stage_go_home->setGoal("Home");
  task.add(std::move(stage_go_home));

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
  stage_open_hand->setGroup(hand_group_name_);
  stage_open_hand->setGoal("Open");
  // current_state_ptr = stage_open_hand.get();
  task.add(std::move(stage_open_hand));

  return task;
}

mtc::Task MTCTaskNode::createSecondTask() {
  mtc::Task task = setupBaseTask("Pick and lift object");
  
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner_ } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.15);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame_;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose("Open");
    stage->setObject("object");
    stage->setAngleDelta(M_PI / 12);
    // stage->setAngleDelta(M_PI);

    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    // define directly using a rotation matrix
    Eigen::Matrix3d R;
    R << 0, 1, 0,
        1, 0, 0,
        0, 0,-1;
    grasp_frame_transform.linear() = R;

    grasp_frame_transform.translation().x() = 0.0;
    grasp_frame_transform.translation().y() = 0.0;
    grasp_frame_transform.translation().z() = 0.13;

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name_)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal("Close");
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame_);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "lift_object");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));

  return task;
}

mtc::Task MTCTaskNode::createThirdTask() {
  mtc::Task task = setupBaseTask("Place object");
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner_ } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      // target_pose_msg.header.frame_id = "object";
      target_pose_msg.header.frame_id = "tag_place";
      target_pose_msg.pose.position.x = 0.0;
      target_pose_msg.pose.position.y = 0.0;
      target_pose_msg.pose.position.z = 0.025;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal("Open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name_)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame_);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }
  return task;
}

mtc::Task MTCTaskNode::createForthTask() {
  mtc::Task task = setupBaseTask("Return to home");
  
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "retreat");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.x = -0.5;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("Home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->doTask();
  rclcpp::shutdown();

  if (spin_thread->joinable())
  {
    spin_thread->join();
  }
  
  return 0;
}