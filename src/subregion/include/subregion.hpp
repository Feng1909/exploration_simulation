#include <iostream>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include "subregion/subregion_map.h"

enum subregion_status
{
    UNEXPLORED,
    EXPLORING,
    EXPLORED,
    OBSTACLE
};

struct subregion_square
{
    double left_down_x, left_down_y;
    double right_up_x, right_up_y;
    enum subregion_status states = UNEXPLORED;
};


nav_msgs::OccupancyGrid map;
bool init_flag = false;
double init_x = 0;
double init_y = 0;
double resolution = 0;
std::vector<subregion_square> squares;
nav_msgs::Odometry car_state_1, car_state_2;

ros::Publisher marker_pub;
ros::Publisher subregion_pub;
ros::Publisher subregion_map_pub;
ros::Subscriber map_sub;
ros::Subscriber car_state_sub_1, car_state_sub_2;

std::vector<double>x,y;

// params
double square_length;

void car_state_callback_1(nav_msgs::Odometry msg) {
    car_state_1 = msg;
}

void car_state_callback_2(nav_msgs::Odometry msg) {
    car_state_2 = msg;
}

void mapCallback(nav_msgs::OccupancyGrid msg) {
    map = msg;
    if (!init_flag) {
        init_x = map.info.origin.position.x;
        init_y = map.info.origin.position.y;
        resolution = map.info.resolution;
        init_flag = true;
    }
}

void gen_subregion() {
    squares.clear();
    subregion::subregion_map subregion_map;
    subregion_map.points.clear();
    subregion_map.status.clear();
    for (auto i=x.begin(); i!=x.end(); i++) {
        for (auto j=y.begin(); j!=y.end(); j++) {
            subregion_square square;
            square.left_down_x = *i;
            square.left_down_y = *j;
            square.right_up_x = *i + square_length;
            square.right_up_y = *j + square_length;

            int left_down_x = floor((square.left_down_x )/resolution);
            int left_down_y = floor((square.left_down_y)/resolution);
            int right_up_x = ceil((square.right_up_x)/resolution);
            int right_up_y = ceil((square.right_up_y)/resolution);
            int unexplored_area = 0;
            for (int i=std::max(left_down_x, 0); i<std::min(right_up_x, int(map.info.width)); i++) {
                for (int j=std::max(left_down_y, 0); j<std::min(right_up_y, int(map.info.height)); j++) {
                    if (map.data[i + j*map.info.width] != -1) {
                        unexplored_area ++;
                    }
                }
            }
            double occupied = double(unexplored_area)/(right_up_x - left_down_x)/(right_up_y - left_down_y);
            if (((car_state_1.pose.pose.position.x-init_x)/resolution > left_down_x && (car_state_1.pose.pose.position.x-init_x)/resolution < right_up_x &&
                (car_state_1.pose.pose.position.y-init_y)/resolution > left_down_y && (car_state_1.pose.pose.position.y-init_y)/resolution < right_up_y) ||
                ((car_state_2.pose.pose.position.x-init_x)/resolution > left_down_x && (car_state_2.pose.pose.position.x-init_x)/resolution < right_up_x &&
                (car_state_2.pose.pose.position.y-init_y)/resolution > left_down_y && (car_state_2.pose.pose.position.y-init_y)/resolution < right_up_y))
                square.states = EXPLORING;
            else if (occupied > 0.9)
                square.states = EXPLORED;
            else if (occupied == 0)
                square.states = OBSTACLE;
            else 
                square.states = UNEXPLORED;
            squares.push_back(square);
            geometry_msgs::Point pos;
            pos.x = (square.left_down_x + square.right_up_x)/2;
            pos.y = (square.left_down_y + square.right_up_y)/2;
            subregion_map.points.push_back(pos);
            subregion_map.status.push_back(occupied);
        }
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "carto_map1";
    marker.ns = "SubRegion";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker.
    marker.scale.x = square_length;
    marker.scale.y = square_length;
    marker.scale.z = 0.01;
    marker.lifetime = ros::Duration();
    std_msgs::ColorRGBA color_explored, color_unexplored, color_obstacle, color_exploring;
    color_explored.r = 0.2;
    color_explored.g = 0.6;
    color_explored.b = 0.3;
    color_explored.a = 0.4;
    color_unexplored = color_explored;
    color_unexplored.a = 0.1;
    color_obstacle = color_explored;
    color_obstacle.r = 0.0;
    color_obstacle.b = 0.0;
    color_obstacle.g = 0.0;
    color_exploring = color_explored;
    color_exploring.a = 0.8;
    for (auto square=squares.begin(); square!=squares.end(); square++) {
        geometry_msgs::Point pos;
        pos.x = (square->left_down_x + square->right_up_x)/2;
        pos.y = (square->left_down_y + square->right_up_y)/2;
        if (square->states == UNEXPLORED)
            marker.colors.push_back(color_unexplored);
        else if (square->states == OBSTACLE)
            marker.colors.push_back(color_obstacle);
        else if (square->states == EXPLORING)
            marker.colors.push_back(color_exploring);
        else
            marker.colors.push_back(color_explored);
        marker.points.push_back(pos);
    }
    subregion_pub.publish(marker);
}

void gen_square() {
    double width = map.info.width;
    double height = map.info.height;
    double start_x = map.info.origin.position.x;
    double start_y = map.info.origin.position.y;
    double resolution = map.info.resolution;
    int num_width = ceil((width*resolution + start_x)/square_length) + \
                    ceil(-start_x/square_length);
    int num_height = ceil((height*resolution + start_y)/square_length) + \
                     ceil(-start_y/square_length);
    int num_square = num_width*num_height;
    visualization_msgs::Marker marker;
    marker.header.frame_id="carto_map1";
    marker.header.stamp = ros::Time::now();
    marker.ns = "grid";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.4;
    marker.color.g = 1.0;
    marker.color.b = 0.8;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration();
    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;

    double left_x = -start_x - ceil(-start_x/square_length)*square_length;
    double left_y = -start_y - ceil(-start_y/square_length)*square_length;
    double end_x = -start_x + ceil((width*resolution + start_x)/square_length)*square_length;
    double end_y = -start_y + ceil((height*resolution + start_y)/square_length)*square_length;

    start_point.x = end_x;
    start_point.y = left_y;
    end_point.x = end_x;
    end_point.y = end_y;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    start_point.x = left_x;
    start_point.y = end_y;
    end_point.x = end_x;
    end_point.y = end_y;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    x.clear();
    y.clear();

    for (double i = -start_x; i < width*resolution; i += square_length) {
        start_point.x = i;
        start_point.y = left_y;
        end_point.x = i;
        end_point.y = end_y;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        x.push_back(i);
    }
    
    for (double i = -start_x; i > -square_length; i -= square_length) {
        start_point.x = i;
        start_point.y = left_y;
        end_point.x = i;
        end_point.y = end_y;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        if (i != -start_x)
            x.push_back(i);
    }

    for (double i = -start_y; i < height*resolution; i += square_length) {
        start_point.x = left_x;
        start_point.y = i;
        end_point.x = end_x;
        end_point.y = i;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        y.push_back(i);
    }

    for (double i = -start_y; i > -square_length; i -= square_length) {
        start_point.x = left_x;
        start_point.y = i;
        end_point.x = end_x;
        end_point.y = i;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        if (i != -start_y)
            y.push_back(i);
    }

    marker_pub.publish(marker);
}