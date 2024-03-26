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
ros::Subscriber subregion_map_sub;
ros::Subscriber car_state_sub_1, car_state_sub_2;
ros::Publisher rrt_planner_vis_pub;

nav_msgs::Odometry car_state_1, car_state_2;
nav_msgs::OccupancyGrid map;
geometry_msgs::Point map_origin;
geometry_msgs::Point target;
float map_width, map_height;
float stride = 0.5;
float square_length = 10;
visualization_msgs::Marker tree_vis;
subregion::subregion_map subregion_map;
subregion::subregion subregion_target;

std::vector<geometry_msgs::Point> path_points;

void subregion_map_callback(subregion::subregion_map msg) {
  subregion_map = msg;
  for (int i=0; i<subregion_map.subregions.size(); i++) {
    if (subregion_map.subregions[i].localization.x == 0 && \
        subregion_map.subregions[i].localization.y == 0) {
      subregion_target = subregion_map.subregions[i];
      break;
    }
  }
}

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

//gridValue function:
//returns grid value at location Xp 
//map data:  100 occupied   -1 unknown    0 free
int gridValue(nav_msgs::OccupancyGrid &mapData, geometry_msgs::Point Xp)
{

  float resolution=mapData.info.resolution;
  float Xstartx=mapData.info.origin.position.x;
  float Xstarty=mapData.info.origin.position.y;
  int out;

  float width=mapData.info.width;
  std::vector<signed char> Data=mapData.data;
  // ROS_WARN_THROTTLE(0.5,"Xp[0]: %f; Xp[1]: %f",Xp[0],Xp[1]);
  float indx=(  floor((Xp.y-Xstarty)/resolution)*width)+( floor((Xp.x-Xstartx)/resolution) );
  // ROS_WARN_THROTTLE(0.5,"indx: %f; mapsize:%d",indx,Data.size());
  out=Data[int(indx)];
  return out;
}

// ObstacleFree function
//return value: -1: unknown   0: obstacle    1: free
int ObstacleFree(geometry_msgs::Point xnear, geometry_msgs::Point &xnew, nav_msgs::OccupancyGrid mapsub)
{
  float rez=float(mapsub.info.resolution)*20;
  float stepz=int(ceil(euclideanDistance(xnew,xnear))/rez);
  geometry_msgs::Point xi=xnear;
  int obs=0;//obstacle 
  int unk=0;//unknown
  geometry_msgs::Point p;
  for (int c=0;c<stepz;c++)
  {
    xi=Steer(xi,xnew,rez);
    if (gridValue(mapsub,xi) >= 50){ obs=1; }
    if (gridValue(mapsub,xi) ==-1){ unk=1;	break;}
  }
  int out=0;
  xnew=xi;
  if (unk==1){  out=-1;}
    
  if (obs==1){  out=0;}
      
  if (obs!=1 && unk!=1){   out=1;}
  
  return out;
}

void rrt_expand() {
	float xr,yr;
	// Sampling random point 
	// Mersenne Twister generator, faster than rand()
	std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.5, 0.5);
  geometry_msgs::Point x_rand;
  geometry_msgs::Point x_nearest;
  geometry_msgs::Point x_new;
  int num = 0;
  bool flag = false;
  while (true && ros::ok()) {
    ros::Time time1 = ros::Time::now();
    if (flag) break;
    // std::cout<<"num: "<<num<<std::endl;
    if (num >= 100) break;
    xr = dis(gen) * square_length + subregion_target.pos.x;
    yr = dis(gen) * square_length + subregion_target.pos.y;
    std::cout<<subregion_target.pos.x<<" "<<subregion_target.pos.y<<" "<<square_length<<" "<<xr<<" "<<yr<<std::endl;
    x_rand.x = xr;
    x_rand.y = yr;
    x_nearest = Nearest(path_points,x_rand);
    ros::Time time2 = ros::Time::now();
    x_new = Steer(x_nearest,x_rand,stride);
    if (x_new.x < subregion_target.left_down.x || x_new.x > subregion_target.right_up.x || \
        x_new.y < subregion_target.left_down.y || x_new.y > subregion_target.right_up.y) {
      continue;
    }
    ros::Time time3 = ros::Time::now();
	  // ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
    int det_check=ObstacleFree(x_nearest, x_new, map);
    ros::Time time4 = ros::Time::now();
    // std::cout<<"time1: "<<(time2 - time1).toSec()<<std::endl;
    // std::cout<<"time2: "<<(time3 - time2).toSec()<<std::endl;
    // std::cout<<"time3: "<<(time4 - time3).toSec()<<std::endl;
    // std::cout<<x_nearest<<std::endl<<x_new<<std::endl;
    path_points.push_back(x_new);
    geometry_msgs::Point x_new_vis=x_new, x_nearest_vis=x_nearest;
    x_new_vis.x -= map_origin.x;
    x_new_vis.y -= map_origin.y;
    x_nearest_vis.x -= map_origin.x;
    x_nearest_vis.y -= map_origin.y;
    tree_vis.points.push_back(x_nearest_vis);
    tree_vis.points.push_back(x_new_vis);
    rrt_planner_vis_pub.publish(tree_vis);
    switch (det_check)
    {
    case 1:
      num ++;
      break;
    
    case -1:
      flag = true;
      break;
    
    case 0:
      num ++;
      break;
    
    default:
      break;
    }
  }
  if (num < 100) {
  }
  else {
    ROS_INFO("Finish exploring the subregion: ");
  }
}
