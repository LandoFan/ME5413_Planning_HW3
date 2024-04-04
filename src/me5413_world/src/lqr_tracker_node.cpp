#include "me5413_world/math_utils.hpp"
#include "me5413_world/lqr_tracker_node.hpp"
#include <Eigen/Dense>

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double LQR_K; // Gain for LQR controller
bool PARAMS_UPDATED;

void dynamicParamCallback(const me5413_world::lqr_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // LQR
  LQR_K = config.LQR_K;

  PARAMS_UPDATED = true;
}

LqrTrackerNode::LqrTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &LqrTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &LqrTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  // LQR Controller Initialization
  Eigen::MatrixXd A(2, 2); // State matrix
  Eigen::MatrixXd B(2, 1); // Control matrix
  Eigen::MatrixXd Q(2, 2); // State cost matrix
  Eigen::MatrixXd R(1, 1); // Control cost matrix

  A << 3, 1,
       1, 3;
  
  B << 1,
       1;

  Q << 1, 0,
       0, 1;
  
  R << 1;

  // Create an instance of LQR controller
  control::LQR lqr_(A, B, Q, R);

    // Define the state vector (example)
  Eigen::VectorXd state(2);
  state << 0, 0; // Example state vector

    // Compute control input using LQR controller
  Eigen::VectorXd control_input = lqr_.computeControl(state);

}

void LqrTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  this->pose_world_goal_ = path->poses[11].pose;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  return;
}

void LqrTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

geometry_msgs::Twist LqrTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  // Compute error (position error only for simplicity)
  double error_x = pose_goal.position.x - odom_robot.pose.pose.position.x;
  double error_y = pose_goal.position.y - odom_robot.pose.pose.position.y;

  // Compute control input using LQR controller
  Eigen::VectorXd state(2);
  state << error_x, error_y;

  Eigen::VectorXd control_input = -lqr_.computeControl(state);

  // Create Twist message for publishing
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = control_input(0); // Assuming control_input(0) is linear velocity
  cmd_vel.angular.z = control_input(1); // Assuming control_input(1) is angular velocity

  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lqr_tracker_node");
  me5413_world::LqrTrackerNode lqr_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
