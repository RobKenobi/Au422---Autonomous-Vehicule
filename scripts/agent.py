#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rospy

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Path
import tf

class Agent:
    def __init__(self):
        self.listener = tf.TransformListener()

        self.robot_pose = Pose2D()
        self.pos = (self.robot_pose.x, self.robot_pose.y)
        self.orient = self.robot_pose.theta
        self.goal_received, self.reached = False, False

        self.path_sub = rospy.Subscriber(
            "path", Path, self.plannerCb, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.timer_pose = rospy.Timer(rospy.Duration(0.5), self.poseCb)
        self.timer_follower = rospy.Timer(
            rospy.Duration(0.1), self.moveToGoal)  # 0.1 sec

        self.linear = 0
        self.angular = 0

        self.path = list()

        self.error_integration = []
        self.point_reached = True
        self.point_to_reach = None

    def poseCb(self, event):
        """ Get the current position of the robot each 500ms """
        try:
            trans, rot = self.listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0))
            self.robot_pose.x = trans[0]
            self.robot_pose.y = trans[1]
            self.pos = (self.robot_pose.x, self.robot_pose.y)
            self.orient = self.robot_pose.theta
            self.robot_pose.theta = tf.transformations.euler_from_quaternion(rot)[
                2]
            print(
                f"Robot's pose: {self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}, {self.robot_pose.theta:.2f}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Could not transform /base_footprint to /map")

    def plannerCb(self, msg):
        self.reached, self.goal_received = False, True
        list_PoseStamped = msg.poses
        self.path = list(map(lambda pose: (pose.pose.position.x,
                         pose.pose.position.y), list_PoseStamped))
        print(f"Chemin agent.py : {self.path}")
        # Remove robot position if in the list
        robot_pose = (self.robot_pose.x, self.robot_pose.y)
        if robot_pose in self.path:
            self.path.remove(robot_pose)

        self.point_to_reach = self.path.pop(0)
        self.point_reached = False

    def moveToGoal(self, event):
        if not self.reached and self.goal_received:

            # Strategy

            if len(self.error_integration) == 80:
                self.error_integration.pop(0)

            # We import the actual position of the robot
            [x, y, yaw] = [self.robot_pose.x,
                           self.robot_pose.y, 
                           self.robot_pose.theta]


            print("La position du robot : ", x, y)
            if math.dist((x, y), self.point_to_reach) < 0.1:
                self.angular = 0
                self.linear = 0
                time.sleep(0.1)
                if len(self.path):
                    self.point_to_reach = self.path.pop(0)

                else:
                    self.reached = True

            else:
                dx = self.point_to_reach[0] - x
                dy = self.point_to_reach[1] - y


                self.linear = 0
                self.angular = 0

                # We compute the angle between the reference of the robot and the goal
                ob = math.atan2(dy, dx)

                angle_error = (ob - yaw)
                if angle_error >= math.pi:
                    angle_error = angle_error - 2*math.pi

                self.error_integration.append(angle_error)

                self.angular = 0.8 * angle_error# + 0.001 * sum(self.error_integration)
                print(f"Angle error : {angle_error}")
                if abs(angle_error) < 0.1:
                    
                    self.linear = 1 - 0.5*abs(angle_error)/0.1
                    


            # End of strategy
            print(
                f"Commandes : {round(self.linear,4)} {round(self.angular,4)}")
            self.send_velocities()

    def send_velocities(self):
        self.linear = self.constraint(self.linear, minimum=-2.0, maximum=2.0)
        self.angular = self.constraint(self.angular)

        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear
        cmd_vel.angular.z = self.angular
        self.vel_pub.publish(cmd_vel)

    def constraint(self, val, minimum=-1.0, maximum=1.0):
        if val < minimum:
            return minimum
        if val > maximum:
            return maximum
        return val


if __name__ == "__main__":
    rospy.init_node("agent_node", anonymous=True)

    node = Agent()

    rospy.spin()
