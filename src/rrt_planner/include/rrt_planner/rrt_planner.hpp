#pragma once
#include "subregion/subregion_map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <time.h>
#include <random>
#include <sstream>

ros::Subscriber map_sub;
ros::Subscriber car_state_sub_1, car_state_sub_2;
ros::Publisher rrt_planner_vis_pub;

nav_msgs::Odometry car_state_1, car_state_2;
nav_msgs::OccupancyGrid map;
geometry_msgs::Point map_origin;
float map_width, map_height;
float stride = 0.5;
visualization_msgs::Marker tree_vis;

std::vector<geometry_msgs::Point> path_points;


void car_state_callback_1(nav_msgs::Odometry msg) {
    car_state_1 = msg;
}

void car_state_callback_2(nav_msgs::Odometry msg) {
    car_state_2 = msg;
}

void mapCallback(nav_msgs::OccupancyGrid msg) {
    map = msg;
	map_width = map.info.width * map.info.resolution;
	map_height = map.info.height * map.info.resolution;
	map_origin = map.info.origin.position;
}

float euclideanDistance(geometry_msgs::Point x, geometry_msgs::Point y) {
    return sqrt(pow(x.x - y.x, 2) + pow(x.y - y.y, 2));
}

//Nearest function: 
//find the nearest vertex from x_rand
geometry_msgs::Point Nearest(std::vector<geometry_msgs::Point> X,geometry_msgs::Point x_rand)
{
  float min=euclideanDistance(X[0],x_rand);
  int indx=0;
  for (int c=1;c<X.size();c++)
  {
    if (euclideanDistance(X[c],x_rand)<min)
    {
      min=euclideanDistance(X[c],x_rand);
      indx=c;
    }
  }
  return X[indx];
}

//Steer function
//Return x_new, which is a new point on the line connecting x_nearst and x_rand
//distance from x_nearst to x_new is stride. 
geometry_msgs::Point Steer(geometry_msgs::Point x_nearest, geometry_msgs::Point x_rand, float stride)
{
  geometry_msgs::Point x_new;
  float norm=euclideanDistance(x_nearest,x_rand);
  x_new.x = x_nearest.x+(stride/(norm+1e-4))*(x_rand.x-x_nearest.x);
  x_new.y = x_nearest.y+(stride/(norm+1e-4))*(x_rand.y-x_nearest.y);
  return x_new;
}

void rrt_expand() {
	float xr,yr;
	// Sampling random point 
	// Mersenne Twister generator, faster than rand()
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);
	xr = dis(gen) * map_width + map_origin.x;
	yr = dis(gen) * map_height + map_origin.y;
    // std::cout<<xr<<" "<<yr<<" "<<dis(gen)<<std::endl;
    geometry_msgs::Point x_rand;
    x_rand.x = xr;
    x_rand.y = yr;
	geometry_msgs::Point x_nearest = Nearest(path_points,x_rand);
	geometry_msgs::Point x_new = Steer(x_nearest,x_rand,stride);
    std::cout<<x_new<<std::endl;
    path_points.push_back(x_new);
    // tree_vis.points.push_back(tree_vis.points.back());
    tree_vis.points.push_back(x_nearest);
    tree_vis.points.push_back(x_new);
    rrt_planner_vis_pub.publish(tree_vis);
}
