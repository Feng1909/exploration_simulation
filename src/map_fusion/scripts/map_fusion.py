#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import OccupancyGrid
from time import time

map1 = OccupancyGrid()
map2 = OccupancyGrid()

def map1_callback(msg):
    global map1
    map1 = msg

def map2_callback(msg):
    global map2
    map2 = msg

def map_fusion(map_pub:rospy.Publisher):
    global map1, map2
    map = OccupancyGrid()
    map.header.frame_id = 'map'
    map.info.resolution = map1.info.resolution
    map.info.width = max(map1.info.width, map2.info.width)
    map.info.height = max(map1.info.height, map2.info.height)
    map.info.origin = map1.info.origin
    map.data = [-1 for i in range(map.info.width * map.info.height)]
    t1 = time()
    print(len(map1.data), len(map2.data))
    ## map1
    for map_height in range(map.info.height):
        if map_height >= map1.info.height:
            break
        for map_width in range(map.info.width):
            if map_width >= map1.info.width:
                break
            if map.data[map_height * map.info.width + map_width] == -1:
                map.data[map_height * map.info.width + map_width] = map1.data[map_height * map1.info.width + map_width]
            elif map1.data[map_height * map1.info.width + map_width] != -1:
                map.data[map_height * map.info.width + map_width] = \
                    min(map.data[map_height * map.info.width + map_width], \
                        map1.data[map_height * map1.info.width + map_width])
            if 0 < map.data[map_height * map.info.width + map_width] < 30:
                map.data[map_height * map.info.width + map_width] = 0
    
    ## map2
    for map_height in range(map.info.height):
        if map_height >= map2.info.height:
            break
        for map_width in range(map.info.width):
            if map_width >= map2.info.width:
                break
            if map.data[map_height * map.info.width + map_width] == -1:
                map.data[map_height * map.info.width + map_width] = map2.data[map_height * map2.info.width + map_width]
            elif map2.data[map_height * map2.info.width + map_width] != -1:
                map.data[map_height * map.info.width + map_width] = \
                    min(map.data[map_height * map.info.width + map_width], \
                        map2.data[map_height * map2.info.width + map_width])
            if 0 < map.data[map_height * map.info.width + map_width] < 30:
                map.data[map_height * map.info.width + map_width] = 0
    
    t2 = time()
    print('time: ', t2-t1)

    # map.info.width = 5
    # map.info.height = 3
    # map.info.resolution = 1
    # map.info.origin.position.x = 0
    # map.info.origin.position.y = 0
    # map.data = [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,-1]
    # print(len(map.data))
    map_pub.publish(map)

if __name__ == '__main__':
    # 定义节点
    rospy.init_node('map_fusion')
    # 订阅地图
    rospy.Subscriber('/robot1/map', OccupancyGrid, map1_callback)
    rospy.Subscriber('/robot2/map', OccupancyGrid, map2_callback)

    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

    rate = rospy.Rate(5)
    while(not rospy.is_shutdown()):
        map_fusion(map_pub)
        rate.sleep()

    rospy.spin()
