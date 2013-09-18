#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>


using namespace ros;

class Projector {
private:
	Publisher project_pub;
	Subscriber sub;
	NodeHandle nh;
	laser_geometry::LaserProjection projector_;
	tf::TransformListener listener_;
	void fancyScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
public:
	Projector();
};

Projector::Projector(): nh("~"){
  // We're going to publish commands for the robot
  this->project_pub = this->nh.advertise<sensor_msgs::PointCloud>("/patch_cloud", 1);
  this->sub = this->nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Projector::scanCallback, this);
}
void Projector::fancyScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  //listener_.waitForTransform("/base_laser_link", "/odom",scan_in->header.stamp, Duration(10.0));
  this->projector_.transformLaserScanToPointCloud("/base_laser_link",*scan_in, cloud,this->listener_);
  cloud.header.stamp = scan_in->header.stamp;
  this->project_pub.publish(cloud);
}
void Projector::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  this->projector_.projectLaser(*scan_in, cloud);
  cloud.header.stamp = scan_in->header.stamp;
  this->project_pub.publish(cloud);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "projector");
  // Always need a node handle
  //NodeHandle node("~");
  Projector p;
  spin();
  return 0;
}
