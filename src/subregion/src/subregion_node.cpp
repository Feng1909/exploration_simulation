#include <subregion.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "subregion_node");
    ros::NodeHandle nh;
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/subregion_marker", 1);
    ros::Rate rate(10);
    nh.getParam("square_length", square_length);
    while (ros::ok()) {
        ros::spinOnce();
        gen_square();
        rate.sleep();
    }
}