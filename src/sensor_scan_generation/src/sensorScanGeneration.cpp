#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cstring>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

string state_estimation_topic = "/state_estimation1";
string registered_scan_topic = "/registered_scan1";
string state_estimation_at_scan_topic = "/state_estimation_at_scan1";
string sensor_scan_topic = "/sensor_scan1";
string odometryIn_header_frame_id = "map1";
string odometryIn_child_frame_id = "sensor_scan1";
string laser_scan_topic = "/laser_scan1";
string laser_header_frame_id = "vehicle1";

bool newTransformToMap = false;

nav_msgs::Odometry odometryIn;
ros::Publisher *pubOdometryPointer = NULL;
tf::StampedTransform transformToMap;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

ros::Publisher pubLaserCloud;
ros::Publisher pubLaserScan;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  tf::Vector3 vec;

  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    vec = transformToMap.inverse() * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();
    if (p1.z > 0.4 || p1.z < 0.0) continue;

    laserCLoudInSensorFrame->points.push_back(p1);
  }

  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = odometryIn_header_frame_id;
  odometryIn.child_frame_id = odometryIn_child_frame_id;
  pubOdometryPointer->publish(odometryIn);

  transformToMap.stamp_ = laserCloud2->header.stamp;
  transformToMap.frame_id_ = odometryIn_header_frame_id;
  transformToMap.child_frame_id_ = odometryIn_child_frame_id;
  tfBroadcasterPointer->sendTransform(transformToMap);

  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = odometryIn_child_frame_id;
  // scan_data.header.frame_id = laser_header_frame_id;
  pubLaserCloud.publish(scan_data);


  // To Laser Scan
  // 创建一个 LaserScan 消息
  sensor_msgs::LaserScan laserScanMsg;

  // 设置 LaserScan 消息的基本信息
  laserScanMsg.header.stamp = ros::Time::now(); // 设置时间戳
  laserScanMsg.header.frame_id = laser_header_frame_id; // 设置坐标系

  // 设置 LaserScan 消息的角度范围和角度步长
  laserScanMsg.angle_min = -M_PI; // 最小角度
  laserScanMsg.angle_max = M_PI; // 最大角度
  //角度分辨率，分辨率越小，转换后的误差越小
  // laserScanMsg.angle_increment = 2 * M_PI / laserCLoudInSensorFrame->size(); // 角度步长
  laserScanMsg.angle_increment = (0.1/180*M_PI); // 角度步长

  // 设置 LaserScan 消息的范围（最小和最大测量距离）
  laserScanMsg.range_min = 0.0; // 最小测量距离
  laserScanMsg.range_max = 100.0; // 最大测量距离

  // int beam_size = ceil(2 * M_PI / laserScanMsg.angle_increment);
  int beam_size = ceil(2 * M_PI / (0.1/180*M_PI));

  //先将所有数据用nan填充
  laserScanMsg.ranges.assign(beam_size, 0.0);
  // laserScanMsg.intensities.assign(beam_size, 0.0);

  // 填充 LaserScan 消息的测量数据
  for (const pcl::PointXYZ& point : laserCLoudInSensorFrame->points)
  {
      // if (point.z > 0.4 || point.z < 0.0) continue;
      // 计算点的极坐标
      double range = sqrt(point.x * point.x + point.y * point.y);
      double angle = atan2(point.y, point.x);

      // int index = (int)((angle - laserScanMsg.angle_min) / laserScanMsg.angle_increment);
      int index = (int)((angle - laserScanMsg.angle_min) / (0.1/180*M_PI));
      // if (index >= 0 && index < beam_size)
      {
        //如果当前内容为nan，则直接赋值
        if (laserScanMsg.ranges[index] == 0.0)
        {
          laserScanMsg.ranges[index] = range;
          // laserScanMsg.intensities[index] = 10.0;
        }
        //否则，只有距离小于当前值时，才可以重新赋值
        else
        {
          if (range < laserScanMsg.ranges[index])
          {
            laserScanMsg.ranges[index] = range;
            // laserScanMsg.intensities[index] = 10.0;
          }
        }
      }

      // // 将测量值添加到 LaserScan 消息中
      // laserScanMsg.ranges.push_back(range);
      // laserScanMsg.intensities.push_back(0.0); // 这里使用0.0作为强度信息，你也可以根据实际情况修改
  }
  pubLaserScan.publish(laserScanMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("state_estimation_topic", state_estimation_topic);
  nhPrivate.getParam("registered_scan_topic", registered_scan_topic);
  nhPrivate.getParam("state_estimation_at_scan_topic", state_estimation_at_scan_topic);
  nhPrivate.getParam("sensor_scan_topic", sensor_scan_topic);
  nhPrivate.getParam("odometryIn_header_frame_id", odometryIn_header_frame_id);
  nhPrivate.getParam("odometryIn_child_frame_id", odometryIn_child_frame_id);
  nhPrivate.getParam("laser_scan_topic", laser_scan_topic);
  nhPrivate.getParam("laser_header_frame_id", laser_header_frame_id);

  // ROS message filters
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(nh, state_estimation_topic, 1);
  subLaserCloud.subscribe(nh, registered_scan_topic, 1);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> (state_estimation_at_scan_topic, 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>(sensor_scan_topic, 2);
  pubLaserScan = nh.advertise<sensor_msgs::LaserScan>(laser_scan_topic, 1);

  ros::spin();

  return 0;
}
