#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

Trajectory control script

author: Ashwin Bose (@atb033)

"""
import sys
import argparse
import yaml
import numpy as np

from generate_trajectory import TrajectoryGenerator

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from math import cos, sin, pi, atan2

class TrajectoryController(object):
    def __init__(self, trajectory_dict):
        self.traj_gen = TrajectoryGenerator(trajectory_dict)

        # Subscribe to positions of the robots
        self.robot_1_pos_subscriber = rospy.Subscriber('/odom/robot_1', Odometry, self.robot_1_odom_cb)
        self.robot_2_pos_subscriber = rospy.Subscriber('/odom/robot_2', Odometry, self.robot_2_odom_cb)

        self.robot_1_pose = None
        self.robot_2_pose = None
        self.control = None

        # Control publisher 
        self.robot_1_cmd_vel_pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)
        self.robot_2_cmd_vel_pub = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(50)

        self.compute_full_control()

    def robot_1_odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw_q = msg.pose.pose.orientation
        theta = euler_from_quaternion([yaw_q.x, yaw_q.y, yaw_q.z, yaw_q.w])[2]

        self.robot_1_pose = np.array([x, y, theta])

    def robot_2_odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw_q = msg.pose.pose.orientation
        theta = euler_from_quaternion([yaw_q.x, yaw_q.y, yaw_q.z, yaw_q.w])[2]

        self.robot_2_pose = np.array([x, y, theta])

    def compute_control(self, pose_real, pose_desired):
        # d = 0.1 # point in front
        # K_p = np.array([[1., 0],[0, 0.1]])

        theta = (pose_real[2] + pi) % (2*pi) - pi
        x_r = pose_real[:2]
        x_d = pose_desired[:2]
        x_err = x_r - x_d

        d = np.linalg.norm(x_err)
        alpha = (atan2(x_err[1], x_err[0]) - theta + pi) % (2*pi) - pi 
        theta_e = (pose_desired[2] - theta + pi ) % (2*pi) - pi

        K_d = 0.5
        K_alpha = 0.2
        K_theta = 0.0
        v = - K_d * cos(alpha) * d
        om = - K_alpha * alpha - K_theta * theta_e

        u = np.array([v, om])

        # K_inv = np.array([[d*cos(theta), d* sin(theta)], [-sin(theta), cos(theta)]]) * (1/d)
        
        # rospy.logwarn_throttle(1, 'pose real: ' + str(pose_real) + '; pose desired: ' + str(pose_desired))

        # # x_err = (pose_real[:2]-pose_desired[:2])

        # # u = -K_p.dot(K_inv.dot(x_err))
        rospy.logwarn_throttle(1, 'x_err: ' + str(x_err)  + ' alpha ' + str(alpha) +  '; u: ' + str(u))
        # TODO: Add stopping condition

        return u

    def compute_full_control(self):
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            self.rate.sleep()
            
            current_time = rospy.Time.now().to_sec()
            t = current_time - start_time

            if not (np.any(self.robot_1_pose) and np.any(self.robot_2_pose)):
                continue

            # get desired pose
            robot_1_desired_pose = self.traj_gen.get_point_vel(t, 'agent0')
            robot_2_desired_pose = self.traj_gen.get_point_vel(t, 'agent1')

            # rospy.logerr_throttle(1, str(robot_1_desired_pose) + '  ' + str(self.robot_1_pose))
            control_1 = self.compute_control(self.robot_1_pose, robot_1_desired_pose)
            control_1_twist = Twist()
            control_1_twist.linear.x = control_1[0]
            control_1_twist.angular.z = control_1[1]
            self.robot_1_cmd_vel_pub.publish(control_1_twist)

            control_2 = self.compute_control(self.robot_2_pose, robot_2_desired_pose)
            control_2_twist = Twist()
            control_2_twist.linear.x = control_2[0]
            control_2_twist.angular.z = control_2[1]
            self.robot_2_cmd_vel_pub.publish(control_2_twist)






def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument("schedule", help="input file containing schedule")
    # args = parser.parse_args()
    
    path = sys.path[0]
    schedule_path = path + '/real_schedule.yaml'
    # Read from input file
    with open(schedule_path, 'r') as schedule_file:
        try:
            schedule = yaml.load(schedule_file)
        except yaml.YAMLError as exc:
            print(exc)
    try:
        rospy.init_node('line_control', anonymous=True)
        t_con = TrajectoryController(schedule)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
