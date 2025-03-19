#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

 

  // Spin up a SingleThreadedExecutor for the current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in RViz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };


  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  std::vector<geometry_msgs::msg::Pose> target_poses; // vector of posses to go to.


  geometry_msgs::msg::Pose pose1;
  pose1.orientation.x = 1.0;
  pose1.orientation.y = 0.0;
  pose1.orientation.z = 0.0;
  pose1.orientation.w = 0.0;
  pose1.position.x = 0.01;
  pose1.position.y = -0.4;
  pose1.position.z = -0.1;
  target_poses.push_back(pose1);


  geometry_msgs::msg::Pose pose2;
  pose1.orientation.x = 1.0;
  pose1.orientation.y = 0.0;
  pose1.orientation.z = 0.0;
  pose1.orientation.w = 0.0;
  pose1.position.x = 0.2;
  pose1.position.y = -0.4;
  pose1.position.z = -0.1;
  target_poses.push_back(pose2);




  

  // ✅ Define the Collision Object (Trolley)
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "trolley";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the trolley size (meters)
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_Y] = 0.712;
    primitive.dimensions[primitive.BOX_X] = 0.875;
    primitive.dimensions[primitive.BOX_Z] = 1.08;

    // Define the pose of the trolley (relative to frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  
    box_pose.position.y = (0.712 / 2) - 0.16;
    box_pose.position.x = 0.0;
    box_pose.position.z = -1.081 / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  auto const collision_object2 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "plate";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the trolley size (meters)
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_Y] = 0.392;
    primitive.dimensions[primitive.BOX_X] = 0.500;
    primitive.dimensions[primitive.BOX_Z] = 0.012;

    // Define the pose of the trolley (relative to frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  
    box_pose.position.y = (0.392 / 2) + 0.13;
    box_pose.position.x = 0.0;
    box_pose.position.z = 0.012 / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  planning_scene_interface.applyCollisionObject(collision_object2);




  

  for (size_t i = 0; i < target_poses.size(); ++i)
  {
    move_group_interface.setPoseTarget(target_poses[i]);
  
    // Set Planner & Constraints
    move_group_interface.setPlannerId("RRTConnectkConfigDefault");
    move_group_interface.setPlanningPipelineId("ompl");
    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setMaxAccelerationScalingFactor(0.6);

    // Convert Pose to Joint Space for Better Motion
    std::vector<double> joint_group_positions;
    move_group_interface.getCurrentState()->copyJointGroupPositions(
        move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator"), 
        joint_group_positions
    );
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Plan & Execute
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // ✅ Execute the Plan
    if (success)
    {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    }
    else
    {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
