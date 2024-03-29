#include <subregion.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "subregion_node");
    ros::NodeHandle nh;
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
    car_state_sub_1 = nh.subscribe<nav_msgs::Odometry>("/state_estimation1", 1, car_state_callback_1);
    car_state_sub_2 = nh.subscribe<nav_msgs::Odometry>("/state_estimation2", 1, car_state_callback_2);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/subregion_marker", 1);
    subregion_pub = nh.advertise<visualization_msgs::Marker>("/subregion", 1);
    subregion_map_pub = nh.advertise<common_msgs::subregion_all>("/subregion_map", 1);
    ros::Rate rate(10);
    nh.getParam("square_length", square_length);
    while (ros::ok()) {
        ros::spinOnce();
        gen_square();
        gen_subregion();
        rate.sleep();
    }
}