#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
import tf
import numpy as np
import cv2

import rrt_dev as rr


class RRT:
    def __init__(self, K=1000, dq=40):
        """ Attributes """
        self.robot_pose = Pose2D()
        self.path = []
        self.listener = tf.TransformListener()
        # TO DO: add your attributes here....

        self.dq = dq
        self.pos = (self.robot_pose.x, self.robot_pose.y)
        self.max_iter = K

        """ Publishers and Subscribers """
        rospy.Timer(rospy.Duration(secs=0.5), self.poseCb)
        rospy.Subscriber("/move_base_simple/goal",
                         PoseStamped, self.goal_pose_cb)
        self.pathPub = rospy.Publisher("/path", Path, queue_size=1)

        """ Load the map and create the related image"""
        self.getMap()

    # **********************************

    def getMap(self):
        """ Call the static_map service and then get the map """
        print("Waiting for map service to be available...")

        rospy.wait_for_service('/static_map')
        print("On est là")
        try:

            get_map = rospy.ServiceProxy('/static_map', GetMap)

            self.map = get_map().map
            self.map_resolution = self.map.info.resolution
            self.map_origin = (-self.map.info.origin.position.x, -
                               self.map.info.origin.position.y)

            self.map_width = self.map.info.width
            self.map_height = self.map.info.height
            print(f"MAP WIDTH {self.map_width}\nMAP HEIGHT {self.map_height}")
            self.img_map = np.array(self.map.data).reshape(
                (self.map_width, self.map_height))
            # np.save("map", self.img_map)
            print("Map received !")
            self.img_map = self.img_map * 64 / 255
            # cv2.imwrite('Map_img.jpg', self.img_map)
            # self.img_out = cv2.imread('Map_img.jpg', 0)
            # cv2.imshow("IMAGE_MAP", 255 - self.img_out)
            # cv2.imshow("IMAGE_MAP", cv2.bitwise_not(self.img_map))
            # cv2.waitKey()

        except rospy.ServiceException as e:
            print(f"Map service call failed: {e}")
            exit()

    # **********************************

    def poseCb(self, event):
        """ Get the current position of the robot each 500ms """
        try:
            trans, rot = self.listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0))
            self.robot_pose.x = trans[0]
            self.robot_pose.y = trans[1]
            self.pos = (self.robot_pose.x, self.robot_pose.y)

            #print(f"Robot's pose: {self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}, {self.robot_pose.theta:.2f}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Could not transform /base_footprint to /map")

    # **********************************

    def goal_pose_cb(self, msg):
        xgoal = msg.pose.position.x
        ygoal = msg.pose.position.y

        self.goal = [xgoal, ygoal]

        print(f"GOAL : \tx = {xgoal}\ty = {ygoal}")
        self.run()

    # **********************************

    def m2p(self, x, y):

        x = int((x + self.map_origin[0]) * 1 / self.map_resolution)
        y = self.map_height - \
            int((-y + self.map_origin[1]) * 1 / self.map_resolution)
        print(x, y)
        return (x, y)

    def run(self):
        init_node = rr.Node(self.m2p(self.pos[0], self.pos[1]))
        goal_node = rr.Node(self.m2p(self.goal[0], self.goal[1]))
        print(goal_node.getPos())
        _map = np.array(self.map.data)
        _map = np.where(_map != 0, 1, 0)
        _map = _map.reshape((self.map_height, self.map_width))

        dq = self.dq
        robot_size = 10
        max_iter = self.max_iter

        P = rr.Path(init_node, goal_node, _map, dq, robot_size, max_iter)
        P.generate(optimize=False, smooth=False)
        self.path = P.getPath("original",init=True)
        print(self.path, file=sys.stderr)

        self.publishPath()

    # **********************************

    def publishPath(self):
        """ Send the computed path so that RVIZ displays it """
        # TODO - Transform the waypoints from pixels coordinates to meters in the map frame
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        path_RVIZ = []
        for pose_img in self.path:
            pose = PoseStamped()
            # self.image_pos = (1 / self.map_resolution * self.map_origin[0], -1 / self.map_resolution * self.map_origin[1] + self.map_height)

            pose.pose.position.x = self.map_resolution * \
                pose_img[0] - self.map_origin[0]
            pose.pose.position.y = self.map_resolution * \
                (pose_img[1]-self.map_height)+self.map_origin[1]
            path_RVIZ.append(pose)
        msg.poses = path_RVIZ
        self.pathPub.publish(msg)


if __name__ == '__main__':
    # DO NOT TOUCH
    rospy.init_node("RRT", anonymous=True)

    rrt = RRT(K=100000)

    rospy.spin()
