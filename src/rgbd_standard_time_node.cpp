#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>

using namespace std;

sensor_msgs::CompressedImage msg_rgbd, msg_infra1, msg_infra2, msg_depth;
sensor_msgs::Image msg_ouster;

ros::Publisher rgbd_pub, infra1_pub, infra2_pub, depth_pub, ouster_pub;

void rgbd_cb(const sensor_msgs::CompressedImageConstPtr &msg){
  msg_rgbd.header.stamp=ros::Time::now();
  //msg_rgbd.header.stamp=ros::Time(new_time_sec);
  msg_rgbd.data=msg->data;
  msg_rgbd.format=msg->format;
  cout<<"rgbd_time: "<<msg_rgbd.header.stamp<<endl;
  rgbd_pub.publish(msg_rgbd);
}

void infra1_cb(const sensor_msgs::CompressedImageConstPtr &msg){
  msg_infra1.header.stamp=ros::Time::now();
  //msg_infra1.header.stamp=ros::Time(new_time_sec);
  msg_infra1.data=msg->data;
  msg_infra1.format=msg->format;
  infra1_pub.publish(msg_infra1);
}

void infra2_cb(const sensor_msgs::CompressedImageConstPtr &msg){
  msg_infra2.header.stamp=ros::Time::now();
  //msg_infra2.header.stamp=ros::Time(new_time_sec);
  msg_infra2.data=msg->data;
  msg_infra2.format=msg->format;
  infra2_pub.publish(msg_infra2);
}


void depth_cb(const sensor_msgs::CompressedImageConstPtr &msg){
  msg_depth.header.stamp=ros::Time::now();
  //msg_depth.header.stamp=ros::Time(new_time_sec);
  msg_depth.data=msg->data;
  msg_depth.format=msg->format;
  depth_pub.publish(msg_depth);
}




int main(int argc, char **argv){
  ros::init(argc, argv, "rgbd_standard_time_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  rgbd_pub=nh.advertise<sensor_msgs::CompressedImage>("/rgbd_standard_time/camera/color/image_raw/compressed",10);
  infra1_pub=nh.advertise<sensor_msgs::CompressedImage>("/rgbd_standard_time/camera/infra1/image_rect_raw/compressed",10);
  infra2_pub=nh.advertise<sensor_msgs::CompressedImage>("/rgbd_standard_time/camera/infra2/image_rect_raw/compressed",10);
  depth_pub=nh.advertise<sensor_msgs::CompressedImage>("/rgbd_standard_time/camera/aligned_depth_to_color/image_raw/compressedDepth",10);

  cout<<"main running"<<endl;

  ros::Subscriber rgbd_sub=nh.subscribe("/camera/color/image_raw/compressed", 10, rgbd_cb);
  ros::Subscriber infra1_sub=nh.subscribe("/camera/infra1/image_rect_raw/compressed", 10, infra1_cb);
  ros::Subscriber infra2_sub=nh.subscribe("/camera/infra2/image_rect_raw/compressed", 10, infra2_cb);
  ros::Subscriber rgbd_test_sub=nh.subscribe("/camera/aligned_depth_to_color/image_raw/compressedDepth", 10, depth_cb);





  ros::spin();
  return 0;
}
