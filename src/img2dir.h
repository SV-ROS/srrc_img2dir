// img2dir.h

#ifndef IMG2DIR_H
#define IMG2DIR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include "std_srvs/Empty.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "obj_detector.h"
#include "utils.h"

namespace img2dir
{

class Img2dir
{
  Nh& nh_;

  image_geometry::PinholeCameraModel cam_model_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher pub_;

  ros::ServiceServer srv_switch_on_;
  ros::ServiceServer srv_switch_off_;
  bool is_on_;


  tf::TransformListener tf_listener_;
  geometry_msgs::Vector3Stamped dir_msg_;
  ros::Publisher target_dir_pub_;
  CvFont font_;

  rs_obj_detector::ColorClustersParams params_;

public:
  Img2dir(Nh& nh)
    : nh_(nh)
    , it_(nh_.nh_)
    , is_on_(false)
  {
    dir_msg_.vector.x = dir_msg_.vector.y = dir_msg_.vector.z = 0;

    std::string input_image_topic_name        = "usb_head_camera/image_raw";
    std::string input_image_transfer_hint     = "compressed";
    std::string output_image_topic_name       = "clustered_image";
    std::string output_direction_topic_name   = "target_dir";
    updateParams();
    updateParam("camera_use_camera_info",      params_.use_camera_info);
    updateParam("input_image_topic_name",      input_image_topic_name);
    updateParam("input_image_transfer_hint",   input_image_transfer_hint);
    updateParam("output_image_topic_name",     output_image_topic_name);
    updateParam("output_direction_topic_name", output_direction_topic_name);
    printParams();
    printParam("input_image_topic_name",       input_image_topic_name);
    printParam("input_image_transfer_hint",    input_image_transfer_hint);
    printParam("output_image_topic_name",      output_image_topic_name);
    printParam("output_direction_topic_name",  output_direction_topic_name);

    // Subscrive to input video feed and publish output video feed
    //sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    //pub_ = it_.advertise("/image_converter/output_video", 1);
    //std::string image_topic = nh_.resolveName("image");


    if(params_.use_camera_info) {
        camera_sub_ = it_.subscribeCamera(input_image_topic_name, 1, &Img2dir::cameraCb, this, image_transport::TransportHints(input_image_transfer_hint));
    } else {
        image_sub_ = it_.subscribe(input_image_topic_name, 1, &Img2dir::imageCb, this, image_transport::TransportHints(input_image_transfer_hint));
    }
    pub_ = it_.advertise(output_image_topic_name, 1);
    target_dir_pub_ = nh_.nh_.advertise<geometry_msgs::Vector3Stamped>(output_direction_topic_name, 100);

    srv_switch_on_  = nh_.nh_.advertiseService("img2dir_wc/switch_on",  &Img2dir::cb_switch_on,  this);
    srv_switch_off_ = nh_.nh_.advertiseService("img2dir_wc/switch_off", &Img2dir::cb_switch_off, this);

    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
  }

  bool cb_switch_on(std_srvs::Empty::Request&, std_srvs::Empty::Response& ) {
      is_on_ = true;
      return true;
  }
  bool cb_switch_off(std_srvs::Empty::Request&, std_srvs::Empty::Response& ) {
      is_on_ = false;
      return true;
  }

  void cameraCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if(!is_on_)
        return;

    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

//    updateParams();
    static const int RADIUS = 3;
    std::string text;

    rs_obj_detector::ObjDetector obj_detector(input_bridge->image.cols, input_bridge->image.rows);
    obj_detector.clusterize(input_bridge->image, params_);

    obj_detector.colorPixels(input_bridge->image, params_);

    clustering::ClusterIndex best_cluster_index = obj_detector.getBestClusterIndex();
    rs_obj_detector::ObjDetector::Cluster const* best_cluster = (best_cluster_index == -1) ? 0 : &obj_detector.getClusters().getTopCluster(best_cluster_index);
    if(obj_detector.gotObjectCenterInPixels()) {
      cv::Point2d const& targetIn2d = obj_detector.getObjectCenterInPixels();
      cv::Point3d targetIn3d = cam_model_.projectPixelTo3dRay(targetIn2d);

      std::stringstream strstr;
      strstr << "direction: {x: " << targetIn3d.x << ", y: " << targetIn3d.y << ", z: " << targetIn3d.z << " }"
             << ", numOfPixels: " << ((!best_cluster) ? 0 : best_cluster->numOfPoints);
      text = strstr.str();

      cv::circle(image, targetIn2d, RADIUS, CV_RGB(255,0,0), -1);

      dir_msg_.header = image_msg->header;
      dir_msg_.vector.x = targetIn3d.x;
      dir_msg_.vector.y = targetIn3d.y;
      dir_msg_.vector.z = targetIn3d.z;
      target_dir_pub_.publish(dir_msg_);
    } else {
        text = "direction: n/a";
    }
    CvSize text_size;
    int baseline;
    cvGetTextSize(text.c_str(), &font_, &text_size, &baseline);
    CvPoint origin = cvPoint(0, input_bridge->image.rows - RADIUS - baseline - 3);
    cv:putText(image, text.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,0,0));
    ROS_INFO("%s", text.c_str());

    pub_.publish(input_bridge->toImageMsg());
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg)
  {
      ROS_INFO("Img2dir TRACE: entered into imageCb()");
      cv::Mat image;
      cv_bridge::CvImagePtr input_bridge;
      try {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
      }
      catch (cv_bridge::Exception& ex){
        ROS_ERROR("[draw_frames] Failed to convert image");
        return;
      }

    updateParams();
    static const int RADIUS = 3;
    std::string text;
    rs_obj_detector::ObjDetector obj_detector(input_bridge->image.cols, input_bridge->image.rows);
    obj_detector.clusterize(input_bridge->image, params_);
    if(obj_detector.gotObjectCenterInPixels()) {
      cv::Point2d const& targetIn2d = obj_detector.getObjectCenterInPixels();
      cv::Point3d targetIn3d; //fixme = cam_model_.projectPixelTo3dRay(targetIn2d);

      std::stringstream strstr;
      clustering::ClusterIndex bestClusterIndex = obj_detector.getBestClusterIndex();
      strstr << "direction: {x: " << targetIn3d.x << ", y: " << targetIn3d.y << ", z: " << targetIn3d.z << " }"
                << ", numOfPixels: " << ((bestClusterIndex == -1) ? 0 : obj_detector.getClusters().getTopCluster(bestClusterIndex).numOfPoints);
      text = strstr.str();

      cv::circle(image, targetIn2d, RADIUS, CV_RGB(255,0,0), -1);

    } else {
        text = "direction: n/a";
    }
//    CvSize text_size;
//    int baseline;
//    cvGetTextSize(text.c_str(), &font_, &text_size, &baseline);
//    CvPoint origin = cvPoint(0, input_bridge->image.rows - RADIUS - baseline - 3);
//    cv:putText(image, text.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,0,0));
//    ROS_INFO("%s", text.c_str());

    pub_.publish(input_bridge->toImageMsg());
  }

  template<typename t_Param>
  static void printParam(std::string const& param_name, t_Param& param, std::string const& prefix = " * ") {
      Nh::printParam(param_name, param, prefix);
  }

  template<typename t_Param>
  void updateParam(std::string const& param_name, t_Param& param) {
      nh_.updateParam(param_name, param);
  }

  void updateParams() {
    updateParam("clusterizer_color_distance_threshold", params_.colorDistanceThreshold);
    updateParam("clusterizer_black_threshold",          params_.black_threshold);
    updateParam("clusterizer_half_window_size",         params_.halfWindowSize);
    updateParam("best_cluster_max_box_size",            params_.maxBoxSizeInBestCluster);
    updateParam("best_cluster_min_num_of_pixels",       params_.minNumOfPixelsInBestCluster);
    updateParam("visualizer_method",                    params_.pixelColoringMethod);
  }

  void printParams() {
    ROS_INFO("Starting img2dir node with parameters:");
    printParam("clusterizer_color_distance_threshold", params_.colorDistanceThreshold);
    printParam("clusterizer_black_threshold",          params_.black_threshold);
    printParam("clusterizer_half_window_size",         params_.halfWindowSize);
    printParam("best_cluster_max_box_size",            params_.maxBoxSizeInBestCluster);
    printParam("best_cluster_min_num_of_pixels",       params_.minNumOfPixelsInBestCluster);
    printParam("visualizer_method",                    params_.pixelColoringMethod);
    printParam("camera_use_camera_info",               params_.use_camera_info);
  }

};

}
#endif
