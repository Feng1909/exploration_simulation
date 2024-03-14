#include <iostream>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

struct subregion_square
{
    double x, y;
    bool explored = false;
};


nav_msgs::OccupancyGrid map;
bool init_flag = false;
double init_x = 0;
double init_y = 0;
double resolution = 0;
std::vector<subregion_square> squares;

ros::Publisher marker_pub;
ros::Subscriber map_sub;

// params
double square_length;

void mapCallback(nav_msgs::OccupancyGrid msg) {
    map = msg;
    if (!init_flag) {
        init_x = map.info.origin.position.x;
        init_y = map.info.origin.position.y;
        resolution = map.info.resolution;
        init_flag = true;
    }
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
    marker.header.frame_id='carto_map1';
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

    for (double i = -start_x; i < width*resolution; i += square_length) {
        start_point.x = i;
        start_point.y = left_y;
        end_point.x = i;
        end_point.y = end_y;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
    }
    
    for (double i = -start_x; i > -square_length; i -= square_length) {
        start_point.x = i;
        start_point.y = left_y;
        end_point.x = i;
        end_point.y = end_y;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
    }

    for (double i = -start_y; i < height*resolution; i += square_length) {
        start_point.x = left_x;
        start_point.y = i;
        end_point.x = end_x;
        end_point.y = i;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
    }

    for (double i = -start_y; i > -square_length; i -= square_length) {
        start_point.x = left_x;
        start_point.y = i;
        end_point.x = end_x;
        end_point.y = i;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
    }

    marker_pub.publish(marker);
}