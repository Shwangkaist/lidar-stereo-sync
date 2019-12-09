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

sensor_msgs::Image msg_ouster_img;
sensor_msgs::PointCloud2 msg_ouster_pcl2;
ros::Publisher ouster_img_pub, ouster_pcl2_pub;



void ouster_img_cb(const sensor_msgs::ImageConstPtr & msg){
  msg_ouster_img.header.stamp=ros::Time::now();
  msg_ouster_img.data=msg->data;
  msg_ouster_img.encoding=msg->encoding;
  msg_ouster_img.height=msg->height;
  msg_ouster_img.width=msg->width;
  msg_ouster_img.step=msg->step;
  ouster_img_pub.publish(msg_ouster_img);
}

void ouster_pcl2_cb(const sensor_msgs::PointCloud2ConstPtr & msg){
  msg_ouster_pcl2.header.stamp=ros::Time::now();
  msg_ouster_pcl2.data=msg->data;
  msg_ouster_pcl2.fields=msg->fields;
  ouster_pcl2_pub.publish(msg_ouster_pcl2);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "ouster_standard_time_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ouster_img_pub=nh.advertise<sensor_msgs::Image>("/ouster_standard_time/img_node/range_image",10);
  ouster_pcl2_pub=nh.advertise<sensor_msgs::PointCloud2>("/ouster_standard_time/os1_cloud_node/points",10);

  cout<<"main running"<<endl;

  ros::Subscriber ouster_img_sub=nh.subscribe("/img_node/range_image", 10, ouster_img_cb);
  ros::Subscriber ouster_pcl2_sub=nh.subscribe("/os1_cloud_node/points", 10, ouster_pcl2_cb);




  ros::spin();
  return 0;
}
