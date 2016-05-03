// stereo2dir.h

#ifndef STEREO2DIR_H
#define STEREO2DIR_H

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include "ImageClusterizer.h"

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>

#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stereo_msgs/DisparityImage.h>

#include <stereo_image_proc/DisparityConfig.h>
//#include <dynamic_reconfigure/server.h>

//#include "std_srvs/Empty.h"

#include "./libstereo_image_proc/processor.h"

namespace img2dir {



struct StereoFrameProcessor : public FrameProcessor {
  typedef FrameProcessor Base;

  //: Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel stereo_cam_model_;
  stereo_image_proc::StereoProcessor block_matcher_; // contains scratch buffers for block matching
  mutable pixel_traits::XyzMat dense_points_;

  stereo_msgs::DisparityImagePtr disp_msg_;
  std::vector<pixel_traits::DistanceRangeInM> not_bad_cluster_distances_;
  std::vector<clustering::ClusterIndex> candidate_cluster_indices_;

  cv::Mat_<float> distance_image_;

  // Params:
  stereo_image_proc::DisparityConfig disparity_config_;


  StereoFrameProcessor() {
    disp_msg_ = boost::make_shared<stereo_msgs::DisparityImage>();
    //fixme: set up disparity_config_
    onDisparityConfigChanged();
  }

  void makeDisparityImage(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                          const sensor_msgs::ImageConstPtr& r_image_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg) {

      // Update the camera model
      stereo_cam_model_.fromCameraInfo(l_info_msg, r_info_msg);

      // Allocate new disparity image message
      disp_msg_->header         = l_info_msg->header;
      disp_msg_->image.header   = l_info_msg->header;

      // Compute window of (potentially) valid disparities
      int border = block_matcher_.getCorrelationWindowSize() / 2;
      int wtf    = (block_matcher_.getMinDisparity() >= 0)
                 ? border + block_matcher_.getMinDisparity()
                 : std::max(border, -block_matcher_.getMinDisparity());
      int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
      int right  = disp_msg_->image.width - 1 - wtf;
      int top    = border;
      int bottom = disp_msg_->image.height - 1 - border;
      disp_msg_->valid_window.x_offset = left;
      disp_msg_->valid_window.y_offset = top;
      disp_msg_->valid_window.width    = right - left;
      disp_msg_->valid_window.height   = bottom - top;

      // Create cv::Mat views onto all buffers
      const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
      const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

      // Perform block matching to find the disparities
      block_matcher_.processDisparity(l_image, r_image, stereo_cam_model_, *disp_msg_);

      // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
      double cx_l = stereo_cam_model_.left().cx();
      double cx_r = stereo_cam_model_.right().cx();
      if (cx_l != cx_r) {
        cv::Mat_<float> disp_image(disp_msg_->image.height, disp_msg_->image.width,
                                  reinterpret_cast<float*>(&disp_msg_->image.data[0]),
                                  disp_msg_->image.step);
        cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
      }
  }

  void filterCandidatesBySize() {
      makeXyzImage();
      clusterizer_.filterCandidatesByDistance(dense_points_);
      clusterizer_.filterCandidatesByEstimatedSize(mono_cam_model_, pixel_traits::StereoClustersParams());
  }

  void publishDisparityImage(ros::Publisher const& disparity_image_pub) {
      if(disparity_image_pub.getNumSubscribers() < 1)
          return;
      disparity_image_pub.publish(disp_msg_);
  }

  void onDisparityConfigChanged() {
    // Tweak all settings to be valid
    disparity_config_.prefilter_size |= 0x1; // must be odd
    disparity_config_.correlation_window_size |= 0x1; // must be odd
    disparity_config_.disparity_range = (disparity_config_.disparity_range / 16) * 16; // must be multiple of 16

    // check stereo method
    // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
    // concurrently, so this is thread-safe.
    block_matcher_.setPreFilterCap(disparity_config_.prefilter_cap);
    block_matcher_.setCorrelationWindowSize(disparity_config_.correlation_window_size);
    block_matcher_.setMinDisparity(disparity_config_.min_disparity);
    block_matcher_.setDisparityRange(disparity_config_.disparity_range);
    block_matcher_.setUniquenessRatio(disparity_config_.uniqueness_ratio);
    block_matcher_.setSpeckleSize(disparity_config_.speckle_size);
    block_matcher_.setSpeckleRange(disparity_config_.speckle_range);
    if (disparity_config_.stereo_algorithm == stereo_image_proc::Disparity_StereoBM) { // StereoBM
      block_matcher_.setStereoType(stereo_image_proc::StereoProcessor::BM);
      block_matcher_.setPreFilterSize(disparity_config_.prefilter_size);
      block_matcher_.setTextureThreshold(disparity_config_.texture_threshold);
    }
    else if (disparity_config_.stereo_algorithm == stereo_image_proc::Disparity_StereoSGBM) { // StereoSGBM
      block_matcher_.setStereoType(stereo_image_proc::StereoProcessor::SGBM);
      block_matcher_.setSgbmMode(disparity_config_.fullDP);
      block_matcher_.setP1(disparity_config_.P1);
      block_matcher_.setP2(disparity_config_.P2);
      block_matcher_.setDisp12MaxDiff(disparity_config_.disp12MaxDiff);
    }
  }

  void makeXyzImage() {
    // Calculate dense point cloud
    const sensor_msgs::Image& dimage = disp_msg_->image;
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    stereo_cam_model_.projectDisparityImageTo3d(dmat, dense_points_, true);
  }

//  void fillDistances() {
//    candidate_cluster_indices_.resize(0);
//    if(Base::clusterizer_.getNumOfCandidateClusters() == 0)
//      return;

//    // Calculate dense point cloud
//    const sensor_msgs::Image& dimage = disp_msg_->image;
//    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
//    stereo_cam_model_.projectDisparityImageTo3d(dmat, dense_points_, true);

//    not_bad_cluster_distances_.resize(Base::clusterizer_.getNumOfCandidateClusters());
//    for (int32_t u = 0, i = 0; u < dense_points_.rows; ++u) {
//      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
//        int not_bad_cluster_index = Base::clusterizer_.getClusters().getTopCluster(i).pp_cluster_index_;
//        if(not_bad_cluster_index != -1) {
//          const cv::Vec3f& p = dense_points_(u,v);
//          pixel_traits::DistanceInM d = pixel_traits::getDistanceInM(p);
//          not_bad_cluster_distances_[not_bad_cluster_index].add(d);
//        }
//      }
//    }
//    //: review not bad clusters found in mono image processing:
//    for(int not_bad_cluster_index = 0; not_bad_cluster_index < Base::clusterizer_.getNumOfCandidateClusters(); ++not_bad_cluster_index) {
//      clustering::ClusterIndex candidate_claster_index = Base::clusterizer_.getCandidateClusterIndices()[not_bad_cluster_index];

//    }
//  }

//  void makeDistanceImage() {
//    // Calculate dense point cloud
//    const sensor_msgs::Image& dimage = disp_msg_->image;
//    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
//    stereo_cam_model_.projectDisparityImageTo3d(dmat, dense_points_, true);

//    distance_image_ = cv::Mat_<pixel_traits::DistanceInM>(dimage.height, dimage.width, 0.f);
//    for (int32_t u = 0; u < dense_points_.rows; ++u) {
//      for (int32_t v = 0; v < dense_points_.cols; ++v) {
//        const cv::Vec3f& p = dense_points_(u,v);
//        distance_image_(u, v) = pixel_traits::getDistanceInM(p);
//      }
//    }
//  }
};




using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class Stereo2dir : public Mono2dirNodeBase {
  //: Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;

  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publishing:
  ros::Publisher disparity_image_pub_;


  StereoFrameProcessor frame_processor_;

  virtual StereoFrameProcessor& getFrameProcessor() { return frame_processor_; }

public:
  Stereo2dir(Nh& nh)
    : Mono2dirNodeBase(nh)
  {
    updateParams();
    printParams();

    //: Subscribtions:
    //

    bool approx = true;
    updateParam("approximate_sync", approx);
    int queue_size = 50;
    updateParam("queue_size", queue_size);
    if (approx) {
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                     sub_l_image_, sub_l_info_,
                                                     sub_r_image_, sub_r_info_) );
        approximate_sync_->registerCallback(boost::bind(&Stereo2dir::stereoImageCb,
                                                        this, _1, _2, _3, _4));
    } else {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                       sub_l_image_, sub_l_info_,
                                       sub_r_image_, sub_r_info_) );
      exact_sync_->registerCallback(boost::bind(&Stereo2dir::stereoImageCb,
                                                this, _1, _2, _3, _4));
    }

    uint32_t msg_que_size = 1;
    std::string input_image_transfer_hint     = "compressed";
    std::string input_image_topic_prefix      = "/turret_stereo/";
    updateParam("input_image_transfer_hint",   input_image_transfer_hint);
    updateParam("input_image_topic_prefix",    input_image_topic_prefix);

    printParam("input_image_topic_prefix",     input_image_topic_prefix);
    printParam("input_image_transfer_hint",    input_image_transfer_hint);

    image_transport::TransportHints hints(input_image_transfer_hint, ros::TransportHints(), nh_.private_nh_);
    sub_l_image_.subscribe(it_,     input_image_topic_prefix + "left/image_rect",   msg_que_size, hints);
    sub_l_info_ .subscribe(nh_.nh_, input_image_topic_prefix + "left/camera_info",  msg_que_size);
    sub_r_image_.subscribe(it_,     input_image_topic_prefix + "right/image_rect",  msg_que_size, hints);
    sub_r_info_ .subscribe(nh_.nh_, input_image_topic_prefix + "right/camera_info", msg_que_size);

    disparity_image_pub_ = nh_.nh_.advertise<DisparityImage>("disparity_image", 1);
    ROS_INFO("TRACE: entered into Stereo2dir::Stereo2dir()-10");
  }

  void stereoImageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                     const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg) {

      ROS_INFO("TRACE: entered into Stereo2dir::stereoImageCb()");
      //updateParams();
      if(!frame_processor_.processMonoImage(l_image_msg, l_info_msg)) {
          ROS_INFO("TRACE: exitting Stereo2dir::stereoImageCb() due to failure.");
          return;
      }
      frame_processor_.makeDisparityImage(l_image_msg, l_info_msg, r_image_msg, r_info_msg);
      frame_processor_.publishDisparityImage(disparity_image_pub_);

      frame_processor_.filterCandidatesBySize();

      std::string text_info = getFrameProcessor().generateTextSummary();
      ROS_INFO("%s", text_info.c_str());
      getFrameProcessor().publishImageColoredBySampleColor(sample_color_image_pub_);
      getFrameProcessor().publishImageColoredByClusterStatus(cluster_status_image_pub_);



      //fixme: publish direction to target
  }

};

}
#endif
