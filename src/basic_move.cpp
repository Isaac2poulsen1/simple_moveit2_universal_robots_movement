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

        scanner_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/barcode_scanner_pose", 10, std::bind(&MoveItWaypointNode::scannerLocationCallback, this, std::placeholders::_1));

        bottlePose_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/bottlePositions", 10, std::bind(&MoveItWaypointNode::bottlePoseCallback, this, std::placeholders::_1));
            


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

    void scannerLocationCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        scanner_position = msg->poses;
        RCLCPP_INFO(this->get_logger(), "scanner position recieved");
    }

    void bottlePoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bottle_positions = msg->poses;
        RCLCPP_INFO(this->get_logger(), "Bottle Positions recieved");
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

    std::vector<geometry_msgs::msg::Pose> getScannerLocations()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return scanner_position;
    }

    std::vector<geometry_msgs::msg::Pose> getBottlePositions()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return bottle_positions;
    }


private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr scanner_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bottlePose_subscriber;


    std::vector<geometry_msgs::msg::Pose> waypoints_;
    std::vector<geometry_msgs::msg::Pose> scanner_position;
    std::vector<geometry_msgs::msg::Pose> bottle_positions;


    bool waypoints_received_;
    std::mutex mutex_;
};



int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItWaypointNode>();
    auto home_pos = homePose();
    auto target_poses = node->getWaypoints();
   


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

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto trolley = createTrolley(move_group_interface.getPlanningFrame());
    auto plate = createPlate(move_group_interface.getPlanningFrame());

    planning_scene_interface.applyCollisionObject(trolley);
    planning_scene_interface.applyCollisionObject(plate);


   

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




    auto bottle_positions = node->getBottlePositions();
    auto scanner_position = node->getScannerLocations();

    if (scanner_position.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "No scanner position received. Aborting bottle transport.");
    }
    else
    {
        const auto &scanner_pose = scanner_position[0]; // Assume only one scanner

        for (const auto &bottle_pose : bottle_positions)
        {
            // Move to bottle position (simulate pickup)
            move_group_interface.setPoseTarget(bottle_pose);
            auto const [success_pick, plan_pick] = [&move_group_interface] {
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                bool ok = static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);

            }();
            if (success_pick)
            {
                move_group_interface.execute(plan_pick);
                RCLCPP_INFO(node->get_logger(), "Moved to bottle pose.");
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Failed to plan to bottle pose. Skipping.");
                continue;
            }

            // Move to scanner location (simulate drop-off)
            move_group_interface.setPoseTarget(scanner_pose);
            auto const [success_drop, plan_drop] = [&move_group_interface] {
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                bool ok = static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);
            }();
            if (success_drop)
            {
                move_group_interface.execute(plan_drop);
                RCLCPP_INFO(node->get_logger(), "Delivered bottle to scanner.");
                // Wait for scanner to process (4 seconds)
                rclcpp::sleep_for(std::chrono::seconds(4));
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Failed to plan to scanner. Skipping.");
            }
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
