#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, Path
import numpy as np

class PathTrackerNode:
    def __init__(self):
        rospy.init_node('path_tracker_node_LQR', anonymous=True)

        self.sub_robot_odom = rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.robot_odom_callback)
        self.sub_local_path = rospy.Subscriber("/me5413_world/planning/local_path", Path, self.local_path_callback)
        self.pub_cmd_vel = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=1)

        self.robot_frame = "base_link"
        self.world_frame = "world"
        self.odom_world_robot = Odometry()
        self.pose_world_goal = Pose()

        self.target_speed = 0.5

    def gain(self, angle):
        A = np.eye(4)
        B = np.array([[np.cos(angle), 0],
                      [np.sin(angle), 0],
                      [0, 1],
                      [1, 0]])
        Q = np.diag([2, 2, 2, 2])
        R = np.diag([0.1, 0.1])
        P = self.riccati_equation(A, B, Q, R)
        K = np.linalg.inv(R + B.T.dot(P).dot(B)).dot(B.T).dot(P).dot(A)
        return K

    def riccati_equation(self, A, B, Q, R):
        P = Q
        for i in range(100):
            P_new = Q + A.T.dot(P).dot(A) - A.T.dot(P).dot(B).dot(np.linalg.inv(R + B.T.dot(P).dot(B))).dot(B.T).dot(P).dot(A)
            if np.allclose(P, P_new, atol=1e-6):
                break
            P = P_new
        return P

    def compute_lqr_control(self, odom_robot, pose_goal, velocity):
        # Convert quaternions to Euler angles
        orientation_list = [odom_robot.pose.pose.orientation.x, odom_robot.pose.pose.orientation.y,
                            odom_robot.pose.pose.orientation.z, odom_robot.pose.pose.orientation.w]
        _, _, yaw_robot = euler_from_quaternion(orientation_list)

        orientation_list_goal = [pose_goal.orientation.x, pose_goal.orientation.y,
                                 pose_goal.orientation.z, pose_goal.orientation.w]
        _, _, yaw_aim = euler_from_quaternion(orientation_list_goal)

        # Heading error
        heading_error = yaw_robot - yaw_aim 

        # State error
        state_error = np.array([odom_robot.pose.pose.position.x - pose_goal.position.x,
                                odom_robot.pose.pose.position.y - pose_goal.position.y,
                                heading_error,
                                velocity])
        K = self.gain(yaw_robot)
        u = - K.dot(state_error)
        return u

    def local_path_callback(self, path):
        if len(path.poses) > 11:
            self.pose_world_goal = path.poses[11].pose
            cmd_vel = Twist()
            velocity = cmd_vel.linear.x
            velocity_error = velocity - self.target_speed
            
            control = self.compute_lqr_control(self.odom_world_robot, self.pose_world_goal, velocity_error)
            cmd_vel = Twist()
            cmd_vel.linear.x = control[0]
            cmd_vel.angular.z = control[1]
            self.pub_cmd_vel.publish(cmd_vel)

    def robot_odom_callback(self, odom):
        self.world_frame = odom.header.frame_id
        self.robot_frame = odom.child_frame_id
        self.odom_world_robot = odom

if __name__ == '__main__':
    node = PathTrackerNode()
    rospy.spin()
