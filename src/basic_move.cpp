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

        box_location_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/box_location", 10, std::bind(&MoveItWaypointNode::boxLocationCallback, this, std::placeholders::_1));
        

            
            


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

    void boxLocationCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        box_location_ = *msg;
        box_location_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received box location at x=%.3f, y=%.3f, z=%.3f",
                    msg->position.x, msg->position.y, msg->position.z);
    }


    bool waypointsAvailable()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return waypoints_received_;
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

    bool isBoxLocationReceived()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return box_location_received_;
    }

    geometry_msgs::msg::Pose getBoxLocation()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return box_location_;
    }



private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr scanner_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bottlePose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr box_location_subscriber_;


    std::vector<geometry_msgs::msg::Pose> waypoints_;
    std::vector<geometry_msgs::msg::Pose> scanner_position;
    std::vector<geometry_msgs::msg::Pose> bottle_positions;
    geometry_msgs::msg::Pose box_location_;



    bool waypoints_received_;
    std::mutex mutex_;
    bool box_location_received_ = false;

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

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto trolley = createTrolley(move_group_interface.getPlanningFrame());
    auto plate = createPlate(move_group_interface.getPlanningFrame());
    planning_scene_interface.applyCollisionObject(trolley);
    planning_scene_interface.applyCollisionObject(plate);


    // Wait for box location
    int box_timeout = 0;
    while (!node->isBoxLocationReceived() && box_timeout < 10000)
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for box location...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        box_timeout += 500;
    }

    if (!node->isBoxLocationReceived())
    {
        RCLCPP_ERROR(node->get_logger(), "Box location not received. Skipping box generation.");
    }
    else
    {
        auto center = node->getBoxLocation();
        geometry_msgs::msg::Vector3 offset;
        offset.x = 0.0;
        offset.y = 0.0;
        offset.z = 0.0;

        geometry_msgs::msg::Vector3 size;
        size.x = 0.36;
        size.y = 0.36;
        size.z = 0.36;

        auto box = createBoxFromOffset(move_group_interface.getPlanningFrame(), center, offset, size);
        planning_scene_interface.applyCollisionObject(box);
    }




    // STEP 1: Move to home pose
    move_group_interface.setPoseTarget(home_pos);
    auto const [success_home, plan_home] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        bool ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success_home)
    {
        move_group_interface.execute(plan_home);
        RCLCPP_INFO(node->get_logger(), "Moved to home position.");
    }
    else
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to move to home position.");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }





    // STEP 2: Wait for scanner location
    std::vector<geometry_msgs::msg::Pose> scanner_position;
    int scanner_timeout = 0;
    while (scanner_position.empty())
    {
        scanner_position = node->getScannerLocations();
        if (scanner_position.empty())
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for scanner location...");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            scanner_timeout += 500;
        }
    }

    // STEP 3: Wait for bottle positions
    std::vector<geometry_msgs::msg::Pose> bottle_positions;
    int bottle_timeout = 0;
    while (bottle_positions.empty())
    {
        bottle_positions = node->getBottlePositions();
        if (bottle_positions.empty())
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for bottle positions...");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            bottle_timeout += 500;
        }
    }

    const auto &scanner_pose = scanner_position[0]; // Assume one scanner

    // STEP 4: Go through bottle pickup and delivery loop
    for (size_t i = 0; i<bottle_positions.size(); i++)
    {
        const auto &bottle_pose = bottle_positions[i];
        RCLCPP_INFO(node->get_logger(), "Planning for bottle #%zu at [x=%.3f, y=%.3f, z=%.3f]", 
                    i + 1, 
                    bottle_pose.position.x, 
                    bottle_pose.position.y, 
                    bottle_pose.position.z);


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
            rclcpp::sleep_for(std::chrono::seconds(4));
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to plan to scanner. Skipping.");
        }
    }

    // Clean shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
