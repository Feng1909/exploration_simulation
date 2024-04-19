#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from math import cos, sin
import numpy as np
import tf.transformations

robot1_state = Odometry()
robot2_state = Odometry()
resolution = 0.1

def robot1_state_callback(msg: Odometry):
    global robot1_state
    robot1_state = msg

def robot2_state_callback(msg: Odometry):
    global robot2_state
    robot2_state = msg

def laser_scan1_callback(msg: LaserScan):
    global laser1, robot1_state, scan_pub
    laser1 = msg
    yaw = tf.transformations.euler_from_quaternion([robot1_state.pose.pose.orientation.x, robot1_state.pose.pose.orientation.y, robot1_state.pose.pose.orientation.z, robot1_state.pose.pose.orientation.w])[2]
    laser1.angle_min += yaw
    laser1.angle_max += yaw
    gen_map(laser1, robot1_state)

def laser_scan2_callback(msg: LaserScan):
    global laser2, robot2_state
    laser2 = msg
    yaw = tf.transformations.euler_from_quaternion([robot2_state.pose.pose.orientation.x, robot2_state.pose.pose.orientation.y, robot2_state.pose.pose.orientation.z, robot2_state.pose.pose.orientation.w])[2]
    laser2.angle_min += yaw
    laser2.angle_max += yaw

def gen_map(laser: LaserScan, odom: Odometry):
    points = []
    min_x = 9999
    min_y = 9999
    max_x = -9999
    max_y = -9999
    # for angle in range(laser.angle_min, laser.angle_max, laser.angle_increment):
    angle = laser.angle_min
    while (angle <= laser.angle_max):
        x = int((odom.pose.pose.position.x + laser.ranges[int(angle / laser.angle_increment)] * cos(angle))/resolution)
        y = int((odom.pose.pose.position.y + laser.ranges[int(angle / laser.angle_increment)] * sin(angle))/resolution)
        points.append([x, y])
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        angle += laser.angle_increment
    # pic_ori = cv2.Mat(max_x - min_x, max_y - min_y, cv2.CV_8UC1, 255)
    # 生成一张图片
    # pic_ori = 255 * np.ones((int(max_x - min_x+10), int(max_y - min_y+10)), np.uint8)
    row = [255 for i in range(int(max_x - min_x)+10)]
    pic_ori = [row for i in range(int(max_y - min_y)+10)]
    for point in points:
        # print(np.array(pic_ori).shape)
        # print(point[0] - min_x, point[1] - min_y)
        pic_ori[point[1] - min_y][point[0] - min_x] = 0
    cv2.imshow("test", np.array(pic_ori).astype("int8"))
    
    

if __name__ == '__main__':
    # 定义节点
    rospy.init_node('map_fusion')
    rospy.Subscriber('/state_estimation1', Odometry, robot1_state_callback)
    rospy.Subscriber('/state_estimation2', Odometry, robot2_state_callback)
    rospy.Subscriber('/laser_scan1', LaserScan, laser_scan1_callback)
    # rospy.Subscriber('/laser_scan1', LaserScan, laser_scan2_callback)
    scan_pub = rospy.Publisher('/laserscan', LaserScan, queue_size=10)
    rospy.spin()