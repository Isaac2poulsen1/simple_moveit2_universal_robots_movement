#include "static_pos.hpp"

geometry_msgs::msg::Pose homePose() {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = 0.0;
  pose.position.z = 0.4;
  pose.orientation.x = 0.0;
  pose.orientation.y = 1.0;
  pose.orientation.z = 0.0; 
  pose.orientation.w = 0.0;
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
