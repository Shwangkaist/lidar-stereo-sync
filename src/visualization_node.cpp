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

void visualization_cb(const cvpr20url::syncedMsgConstPtr &msg){
  cv::Mat depthColor_mat; // Depth image
  cv::Mat infra1_mat;     // Raw infra 1 image
  cv::Mat infra2_mat;     // Raw infra 2 image
  cv::Mat color_mat;      // Color image
  cv::Mat ousterDepth_mat;
  cv_bridge::CvImagePtr cv_ptr_rgbd;
  cv_bridge::CvImagePtr cv_ptr_ouster;
    ////// RGBD Depth info part

  cv_ptr_rgbd = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  double min, max;
  min = 0.0;
  max = 10000.0;
  cv::convertScaleAbs(cv_ptr_rgbd->image, depthColor_mat,255.0/(max-min),-255.0/(max-min)*min);
  cv::applyColorMap(depthColor_mat, depthColor_mat, 2);


    ////// Ouster Depth info part

  cv_ptr_ouster = cv_bridge::toCvCopy(msg->ouster_depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  ousterDepth_mat = cv_ptr_ouster->image;
  imshow("Ouster image", ousterDepth_mat);



    ////// RGB info part
  color_mat=cv::imdecode(cv::Mat(msg->rgb_image.data),cv::IMREAD_COLOR);



    ////// Infra1 and Infra 2 part

  infra1_mat=cv::imdecode(cv::Mat(msg->infra1_image.data),cv::IMREAD_COLOR);
  infra2_mat=cv::imdecode(cv::Mat(msg->infra2_image.data),cv::IMREAD_COLOR);



    ////// Concat part (For visualization only)

  cv::Mat infra;
  int infra1_row=infra1_mat.rows;
  int infra2_row=infra2_mat.rows;



  if (infra1_row!=infra2_row){

    cout<<"Dimension not equal between infra1 and infra 2"<<endl;
  }
  else if(infra1_mat.empty() || infra2_mat.empty()){
    cout<<"One of infra1_mat and infra2_mat is empty"<<endl;
  }
  else{
    cv::hconcat(infra1_mat, infra2_mat, infra);
  }



  //RGBD Camera concat visualzation
  cv::Mat depth_n_rgb;
  cv::hconcat(depthColor_mat,color_mat,depth_n_rgb);
  cv::Mat final_image;
  cv::vconcat(infra, depth_n_rgb, final_image);
  cv::imshow("Result Image", final_image);
  cv::waitKey(1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  cout<<"main running"<<endl;
  ros::Subscriber filtered_sub = nh.subscribe("/filtered_topic", 10, visualization_cb);


  ros::spin();
  return 0;
}
