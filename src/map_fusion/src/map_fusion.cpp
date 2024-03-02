#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

nav_msgs::OccupancyGrid map1, map2;
geometry_msgs::Point origin_1, origin_2;

void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map1 = *msg;
    if (origin_1.x == -999 || origin_1.y == -999)
    origin_1 = map1.info.origin.position;
}

void map2Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map2 = *msg;
    if (origin_2.x == -999 || origin_2.y == -999)
    origin_2 = map2.info.origin.position;
}

void map_fusion(ros::Publisher pub) {
    if (origin_1.x == -999 || origin_1.y == -999 ||
        origin_2.x == -999 || origin_2.y == -999)
        return;
    ros::Time t1 = ros::Time::now();
    nav_msgs::OccupancyGrid map_fusion;
    int height = map1.info.height;
    int width = map1.info.width;
    double resolution = map1.info.resolution;

    map_fusion.header.frame_id = "map";
    map_fusion.info.resolution = map1.info.resolution;
    map_fusion.info.width = std::max(map1.info.width - int(std::fabs(map1.info.origin.position.x)/resolution),
                                        2+map2.info.width - int(std::fabs(map2.info.origin.position.x)/resolution)) +
                             std::max(std::fabs(map1.info.origin.position.x)/resolution, std::fabs(map2.info.origin.position.x)/resolution-2);
    map_fusion.info.height = std::max(map1.info.height - int(std::fabs(map1.info.origin.position.y)/resolution),
                                        map2.info.height - int(std::fabs(map2.info.origin.position.y)/resolution)) +
                             std::max(std::fabs(map1.info.origin.position.y)/resolution, std::fabs(map2.info.origin.position.y)/resolution);
    map_fusion.info.origin.position.x = std::min(map1.info.origin.position.x, map2.info.origin.position.x+2);
    map_fusion.info.origin.position.y = std::min(map1.info.origin.position.y, map2.info.origin.position.y);
    
    for (int i=0; i<map_fusion.info.height; i++) {
        for (int j=0; j<map_fusion.info.width; j++) {
            map_fusion.data.push_back(-1);
        }
    }

    geometry_msgs::Point bias1, bias2;
    bias1.y = int(-(map_fusion.info.origin.position.y - map1.info.origin.position.y)/resolution);
    bias1.x = int(-(map_fusion.info.origin.position.x - map1.info.origin.position.x)/resolution);
    bias2.y = int(-(map_fusion.info.origin.position.y - map2.info.origin.position.y)/resolution);
    bias2.x = int(-(map_fusion.info.origin.position.x - map2.info.origin.position.x-2)/resolution);
    std::cout<<"bias: "<<bias1.x<<" "<<bias1.y<<std::endl;
    for (int i=0; i<map1.info.height; i++) {
        for (int j=0; j<map1.info.width; j++) {
            if (map_fusion.data[(i+bias1.y)*map_fusion.info.width+j+bias1.x] == -1)
                map_fusion.data[(i+bias1.y)*map_fusion.info.width+j+bias1.x] = map1.data[i*map1.info.width+j];
            else if (map1.data[i*map1.info.width+j] != -1)
                map_fusion.data[(i+bias1.y)*map_fusion.info.width+j+bias1.x] = \
                    std::min(map_fusion.data[(i+bias1.y)*map_fusion.info.width+j+bias1.x], map1.data[i*map1.info.width+j]);
        }
    }
    for (int i=0; i<map2.info.height; i++) {
        for (int j=0; j<map2.info.width; j++) {
            if (map_fusion.data[(i+bias2.y)*map_fusion.info.width+j+bias2.x] == -1)
                map_fusion.data[(i+bias2.y)*map_fusion.info.width+j+bias2.x] = map2.data[i*map2.info.width+j];
            else if (map2.data[i*map2.info.width+j] != -1)
                map_fusion.data[(i+bias2.y)*map_fusion.info.width+j+bias2.x] = \
                    std::min(map_fusion.data[(i+bias2.y)*map_fusion.info.width+j+bias2.x], map2.data[i*map2.info.width+j]);
        }
    }
    std::cout<<"size:"<<map_fusion.data.size()<<std::endl;
    ros::Time t2 = ros::Time::now();
    std::cout<<"cost time: "<<t2-t1<<std::endl;
    pub.publish(map_fusion);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_fusion");
    ros::NodeHandle nh;
    ROS_INFO("Map fusion node has been started");
    ros::Subscriber sub1 = nh.subscribe<nav_msgs::OccupancyGrid>("/robot1/map", 1, map1Callback);
    ros::Subscriber sub2 = nh.subscribe<nav_msgs::OccupancyGrid>("/robot2/map", 1, map2Callback);
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Rate rate(3);

    origin_1.x = -999;
    origin_1.y = -999;
    origin_2.x = -999;
    origin_2.y = -999;
    while (ros::ok())
    {
        map_fusion(pub);
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}