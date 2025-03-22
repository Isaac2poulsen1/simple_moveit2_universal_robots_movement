// collision_objects.hpp
#ifndef COLLISION_OBJECTS_HPP
#define COLLISION_OBJECTS_HPP

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>

// Function to create a collision object (trolley)
moveit_msgs::msg::CollisionObject createTrolley(const std::string& frame_id);

// Function to create a collision object (plate)
moveit_msgs::msg::CollisionObject createPlate(const std::string& frame_id);

#endif // COLLISION_OBJECTS_HPP