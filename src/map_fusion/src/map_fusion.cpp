#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid map1, map2;

void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map1 = *msg;
}

void map2Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map2 = *msg;
}

void map_fusion(ros::Publisher pub) {
    ros::Time t1 = ros::Time::now();
    nav_msgs::OccupancyGrid map_fusion;
    int height = map1.info.height;
    int width = map1.info.width;
    map_fusion.header.frame_id = "map";
    map_fusion.info.resolution = map1.info.resolution;
    map_fusion.info.width = std::max(map1.info.width, map2.info.width);
    map_fusion.info.height = std::max(map1.info.height, map2.info.height);
    map_fusion.info.origin = map1.info.origin;
    for (int i=0; i<map_fusion.info.height; i++) {
        for (int j=0; j<map_fusion.info.width; j++) {
            map_fusion.data.push_back(-1);
        }
    }
    // map_fusion.data.resize(map_fusion.info.width * map_fusion.info.height);
    std::cout<<"size:"<<map_fusion.data.size()<<std::endl;
    for (int i=0; i<height; i++) {
        if (i >= map1.info.height && i>= map2.info.height)
            break;
        for (int j=0; j<width; j++) {
            if (j >= map1.info.width && i >= map2.info.height)
                break;
            if (i < map1.info.height && j < map1.info.width) {
                if (map_fusion.data[i*map_fusion.info.width+j] == -1)
                    map_fusion.data[i*map_fusion.info.width+j] = map1.data[i*map1.info.width+j];
                else if (map1.data[i*map1.info.width+j] != -1)
                    map_fusion.data[i*map_fusion.info.width+j] = \
                        std::min(map_fusion.data[i*map_fusion.info.width+j], map1.data[i*map1.info.width+j]);
            }
            if (i < map2.info.height && j < map2.info.width) {
                if (map_fusion.data[i*map_fusion.info.width+j] == -1)
                    map_fusion.data[i*map_fusion.info.width+j] = map2.data[i*map2.info.width+j];
                else if (map2.data[i*map2.info.width+j] != -1)
                    map_fusion.data[i*map_fusion.info.width+j] = \
                        std::min(map_fusion.data[i*map_fusion.info.width+j], map2.data[i*map2.info.width+j]);
            }

        }
    }
    // for (int i=0; i<height; i++) {
    //     if (i >= map2.info.height)
    //         break;
    //     for (int j=0; j<width; j++) {
    //         if (j >= map2.info.width)
    //             break;
    //         if (map_fusion.data[i*map_fusion.info.width+j] == -1)
    //             map_fusion.data[i*map_fusion.info.width+j] = map2.data[i*map2.info.width+j];
    //         else if (map2.data[i*map2.info.width+j] != -1)
    //             map_fusion.data[i*map_fusion.info.width+j] = \
    //                 std::min(map_fusion.data[i*map_fusion.info.width+j], map2.data[i*map2.info.width+j]);
            
    //     }
    // }
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
    while (ros::ok())
    {
        map_fusion(pub);
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}