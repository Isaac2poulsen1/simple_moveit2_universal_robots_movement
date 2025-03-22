// collision_objects.cpp
#include "collision_objects.hpp"

moveit_msgs::msg::CollisionObject createTrolley(const std::string& frame_id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "trolley";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_Y] = 0.712;
    primitive.dimensions[primitive.BOX_X] = 0.875;
    primitive.dimensions[primitive.BOX_Z] = 1.08;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  
    box_pose.position.y = (0.712 / 2) - 0.16;
    box_pose.position.x = 0.0;
    box_pose.position.z = -1.081 / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

moveit_msgs::msg::CollisionObject createPlate(const std::string& frame_id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "plate";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_Y] = 0.392;
    primitive.dimensions[primitive.BOX_X] = 0.500;
    primitive.dimensions[primitive.BOX_Z] = 0.012;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.y = (0.392 / 2) + 0.13;
    box_pose.position.x = 0.0;
    box_pose.position.z = 0.012 / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}
