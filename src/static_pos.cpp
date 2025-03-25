#include "static_pos.hpp"

geometry_msgs::msg::Pose homePose() {
  geometry_msgs::msg::Pose pose;
  default_pose.orientation.w = 0.0;
  default_pose.orientation.x = 1.0;
  default_pose.position.x = 0.09;
  default_pose.position.y = 0.386;
  default_pose.position.z = 0.4;
  return pose;
}




geometry_msgs::msg::Pose pickPose() {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.3;
  pose.position.y = -0.2;
  pose.position.z = 0.2;
  pose.orientation.x = 0.0;
  pose.orientation.y = 1.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}
