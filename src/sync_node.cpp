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

#include <cvpr20url/syncedMsg.h>
#include <sensor_msgs/Image.h>

using namespace std;

cvpr20url::syncedMsg msg_pub;
ros::Publisher synced_pub;



void allSensordata_cb(const sensor_msgs::ImageConstPtr &msg_depth, const sensor_msgs::CompressedImageConstPtr &msg_infra1,
  const sensor_msgs::CompressedImageConstPtr &msg_infra2, const sensor_msgs::CompressedImageConstPtr &msg_rgb
  , const sensor_msgs::ImageConstPtr &msg_ouster_depth_img, const sensor_msgs::PointCloud2ConstPtr &msg_ouster_pcl2
){



  ////////////////////////////////////////////////////////////
  ////// Publish synchronized messaages into cvpr20::syncedMsg
  ////////////////////////////////////////////////////////////
  msg_pub.infra1_image.data=msg_infra1->data;
  msg_pub.infra1_image.format=msg_infra1->format;

  msg_pub.infra2_image.data=msg_infra2->data;
  msg_pub.infra2_image.format=msg_infra2->format;

  msg_pub.depth_image.data=msg_depth->data;
  msg_pub.depth_image.encoding=msg_depth->encoding;
  msg_pub.depth_image.height=msg_depth->height;
  msg_pub.depth_image.width=msg_depth->width;
  msg_pub.depth_image.step=msg_depth->step;

  msg_pub.rgb_image.data=msg_rgb->data;
  msg_pub.rgb_image.format=msg_rgb->format;

  msg_pub.ouster_depth_image.data=msg_ouster_depth_img->data;
  msg_pub.ouster_depth_image.encoding=msg_ouster_depth_img->encoding;
  msg_pub.ouster_depth_image.height=msg_ouster_depth_img->height;
  msg_pub.ouster_depth_image.width=msg_ouster_depth_img->width;
  msg_pub.ouster_depth_image.step=msg_ouster_depth_img->step;

  msg_pub.ouster_pcl2.data=msg_ouster_pcl2->data;

  synced_pub.publish(msg_pub);


}

int main(int argc, char **argv){
  ros::init(argc, argv, "sync_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  synced_pub=nh.advertise<cvpr20url::syncedMsg>("/synced_topic",10);
  cout<<"main running"<<endl;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/depth",10);
  message_filters::Subscriber<sensor_msgs::CompressedImage> infra1_sub(nh, "/rgbd_standard_time/camera/infra1/image_rect_raw/compressed",10);
  message_filters::Subscriber<sensor_msgs::CompressedImage> infra2_sub(nh, "/rgbd_standard_time/camera/infra2/image_rect_raw/compressed",10);
  message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_sub(nh, "/rgbd_standard_time/camera/color/image_raw/compressed",10);
  message_filters::Subscriber<sensor_msgs::Image> ouster_img_sub(nh, "/ouster_standard_time/img_node/range_image",10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> ouster_pcl2_sub(nh, "/ouster_standard_time/os1_cloud_node/points",10);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, infra1_sub, infra2_sub, rgb_sub, ouster_img_sub, ouster_pcl2_sub);
  sync.registerCallback(boost::bind(&allSensordata_cb, _1, _2, _3, _4, _5, _6));







  ros::spin();
  return 0;
}
