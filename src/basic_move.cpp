#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "collision_objects.hpp"
#include "static_pos.hpp"


#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

class MoveItWaypointNode : public rclcpp::Node
{
public:
    MoveItWaypointNode()
        : Node("basic_move"), waypoints_received_(false)
    {
        // Initialize ROS subscriber to listen for waypoints
        waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints", 10, std::bind(&MoveItWaypointNode::waypointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for waypoints...");
    }

    // Callback function for receiving waypoints
    void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        waypoints_ = msg->poses;
        waypoints_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", waypoints_.size());
    }

    bool waypointsAvailable()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return waypoints_received_;
    }


    std::vector<geometry_msgs::msg::Pose> getWaypoints()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (waypoints_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No waypoints received, using default goal.");

            // Default Goal
            geometry_msgs::msg::Pose default_pose;
            default_pose.orientation.w = 1.0;
            default_pose.position.x = 0.0;
            default_pose.position.y = -0.1;
            default_pose.position.z = -0.1;
            waypoints_.push_back(default_pose);
        }
        return waypoints_;
    }



private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscriber_;
    std::vector<geometry_msgs::msg::Pose> waypoints_;
    bool waypoints_received_;
    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItWaypointNode>();
    auto home_pos = homePose();


    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    auto moveit_visual_tools =
        moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                               move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Load collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto trolley = createTrolley(move_group_interface.getPlanningFrame());
    auto plate = createPlate(move_group_interface.getPlanningFrame());

    // Add objects to the scene
    planning_scene_interface.applyCollisionObject(trolley);
    planning_scene_interface.applyCollisionObject(plate);

    // Wait for waypoints, but apply timeout (e.g., 5 seconds)
    int timeout_counter = 0;
    int max_wait_time = 2000;
    while (!node->waypointsAvailable() && timeout_counter < max_wait_time)
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for waypoints to be published...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        timeout_counter += 500;
    }

    auto target_poses = node->getWaypoints();
    for (const auto &target_pose : target_poses)
    {
        move_group_interface.setPoseTarget(target_pose);

        auto const [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success)
        {
            move_group_interface.execute(plan);
            RCLCPP_INFO(node->get_logger(), "Executed waypoint");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Planning failed!");
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
