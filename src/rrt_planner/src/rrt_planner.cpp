#include <iostream>
#include "rrt_planner/rrt_planner.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_planner_node");
    ros::NodeHandle nh;

    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
    car_state_sub_1 = nh.subscribe<nav_msgs::Odometry>("/state_estimation1", 1, car_state_callback_1);
    car_state_sub_2 = nh.subscribe<nav_msgs::Odometry>("/state_estimation2", 1, car_state_callback_2);

    rrt_planner_vis_pub = nh.advertise<visualization_msgs::Marker>("/rrt_planner_vis", 1);

    ros::Rate rate(1);
    geometry_msgs::Point start;
    start.x = 0;
    start.y = 0;
    tree_vis.header.frame_id = "carto_map1";
    tree_vis.type = 5;
    tree_vis.action = 0;
    tree_vis.scale.x = 0.1;
    tree_vis.scale.y = 0.1;
    tree_vis.scale.z = 0.1;
    tree_vis.color.r = 1;
    tree_vis.color.g = 0.1;
    tree_vis.color.b = 0.2;
    tree_vis.color.a = 1;
    // tree_vis.points.push_back(start);
    // tree_vis.points.push_back(start);
    path_points.push_back(start);
    while(ros::ok()) {
        ros::spinOnce();
        rrt_expand();
        rate.sleep();
    }
}