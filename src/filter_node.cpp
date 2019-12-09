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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cvpr20url/syncedMsg.h>

using namespace std;

cvpr20url::syncedMsg msg_pub_filtered;
ros::Publisher filtered_pub;

void filter_cb(const cvpr20url::syncedMsgConstPtr &msg){
  /////////////////////////////////////////////////
  /////////////////Todo////////////////////////////
  /////////////////////////////////////////////////

  msg_pub_filtered.infra1_image.data=msg->infra1_image.data;
  msg_pub_filtered.infra1_image.format=msg->infra1_image.format;

  msg_pub_filtered.infra2_image.data=msg->infra2_image.data;
  msg_pub_filtered.infra2_image.format=msg->infra2_image.format;

  msg_pub_filtered.depth_image.data=msg->depth_image.data;
  msg_pub_filtered.depth_image.encoding=msg->depth_image.encoding;
  msg_pub_filtered.depth_image.height=msg->depth_image.height;
  msg_pub_filtered.depth_image.width=msg->depth_image.width;
  msg_pub_filtered.depth_image.step=msg->depth_image.step;

  msg_pub_filtered.rgb_image.data=msg->rgb_image.data;
  msg_pub_filtered.rgb_image.format=msg->rgb_image.format;

  msg_pub_filtered.ouster_depth_image.data=msg->ouster_depth_image.data;
  msg_pub_filtered.ouster_depth_image.encoding=msg->ouster_depth_image.encoding;
  msg_pub_filtered.ouster_depth_image.height=msg->ouster_depth_image.height;
  msg_pub_filtered.ouster_depth_image.width=msg->ouster_depth_image.width;
  msg_pub_filtered.ouster_depth_image.step=msg->ouster_depth_image.step;


  msg_pub_filtered.ouster_pcl2.data=msg->ouster_pcl2.data;

  filtered_pub.publish(msg_pub_filtered);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh;
    filtered_pub=nh.advertise<cvpr20url::syncedMsg>("/filtered_topic",10);
    ros::Subscriber synced_sub = nh.subscribe("/synced_topic", 10, filter_cb);


    ros::spin();

    return 0;
}
