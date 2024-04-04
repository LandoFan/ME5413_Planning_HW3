/** path_tracker_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for robot to track a given path
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
// Pure pursuit lookahead distance
double lookahead_distance; 
// double STANLEY_K;
bool PARAMS_UPDATED;
// Angular velocity proportional coefficient
double Kp = 2; 

void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // PID
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  // Stanley
  // STANLEY_K = config.stanley_K;
  // Loadahead distance
  lookahead_distance = config.lookahead_distance;

  PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  this->pub_cmd_vel_.publish(ControlOutput(this->odom_world_robot_, path));
  return;
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

// double PathTrackerNode::computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity)
// {
//   const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

//   return std::min(std::max(stanley_output, -2.2), 2.2);
// }

// find the goal point based on lookahead distance
geometry_msgs::Point PathTrackerNode::LookaheadPoint(const tf2::Vector3& point_robot, const nav_msgs::Path::ConstPtr& path, double lookahead_distance) {
  for (const auto& pose : path->poses) 
  {
    tf2::Vector3 point_path;
    tf2::fromMsg(pose.pose.position, point_path);
    if (tf2::tf2Distance(point_robot, point_path) >= lookahead_distance) 
    {
      return pose.pose.position;
    }
  }
  return path->poses.back().pose.position;
}

// normalize the angle to range [-pi, pi)
double PathTrackerNode::normalizeAngle(double angle) {
  double a = fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

geometry_msgs::Twist PathTrackerNode::ControlOutput(const nav_msgs::Odometry& odom_robot, const nav_msgs::Path::ConstPtr& path)
{
  // Extract robot's orientation and position
  tf2::Quaternion q_robot;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  double roll, pitch, yaw;
  m_robot.getRPY(roll, pitch, yaw);
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);

  // Change velocity based on PID controller
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();
  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }
  // Controller output linear speed control
  cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);

  // Find the goal point and calculate the error
  geometry_msgs::Point lookahead_point = LookaheadPoint(point_robot, path, lookahead_distance);
  double goal = atan2(lookahead_point.y - point_robot.y(), lookahead_point.x - point_robot.x());
  double yaw_error = normalizeAngle(goal - yaw);

  // Controller output angle control
  cmd_vel.angular.z = Kp * yaw_error;

  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
