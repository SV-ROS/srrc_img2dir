// ImageClusterizer.h

#ifndef IMG2DIR_IMAGECLUSTERIZER_H
#define IMG2DIR_IMAGECLUSTERIZER_H

#include <boost/foreach.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

#include "obj_detector.h"
#include "utils.h"

#include <opencv2/highgui/highgui.hpp>

namespace img2dir {

    struct FrameProcessor {
        FrameProcessor() {
            got_cam_model_ = false;
            dir_msg_.vector.x = dir_msg_.vector.y = dir_msg_.vector.z = 0;
        }

        bool processMonoImage(sensor_msgs::Image const& image_msg) {
            TRACE_TO_ROS_INFO("TRACE(FrameProcessor): entered into processMonoImage(image_msg)");
            got_cam_model_ = false;
            try {
              original_image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
              TRACE_TO_ROS_INFO("TRACE(FrameProcessor): entered into processMonoImage(image_msg)-1");
            }
            catch (...){
              ROS_ERROR("[draw_frames] Failed to convert image");
              return false;
            }

            preprocessed_image_ = colorizer_.preprocessImage(original_image_->image
                                                             , params_.preprocess_params_.saturation_threshold_
                                                             , params_.preprocess_params_.brightness_threshold_);
            clusterizer_.clusterizeAndPp(preprocessed_image_, params_.color_cluster_params_);
            TRACE_TO_ROS_INFO("TRACE(FrameProcessor): entered into processMonoImage(image_msg)-done.");
            return true;
        }

        bool processMonoImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
            if(!processMonoImage(*image_msg))
                return false;
            TRACE_TO_ROS_INFO("TRACE(FrameProcessor): entered into processMonoImage(image_msg, info_msg)-0");
            got_cam_model_ = true;
            mono_cam_model_.fromCameraInfo(info_msg);
            //TRACE_TO_ROS_INFO("TRACE(FrameProcessor): entered into processMonoImage(image_msg, info_msg)-1");

    //        clustering::ClusterIndex best_cluster_index = clusterizer_.getBestClusterIndex();
    //        obj_detector::ImageClusterizer::Cluster const* best_cluster
    //                = (best_cluster_index == -1)
    //                ? 0 : &clusterizer_.getClusters().getTopCluster(best_cluster_index);
    //        std::string text;
    //        if(clusterizer_.gotObjectCenterInPixels()) {
    //          targetIn2d_ = clusterizer_.getObjectCenterInPixels();
    //          targetIn3d_ = mono_cam_model_.projectPixelTo3dRay(targetIn2d_);

    //          std::stringstream strstr;
    //          strstr << "direction: {x: " << targetIn3d_.x << ", y: " << targetIn3d_.y << ", z: " << targetIn3d_.z << " }"
    //                 << ", numOfPixels: " << ((!best_cluster) ? 0 : best_cluster->numOfPoints);
    //          text = strstr.str();

    //          colorizer_.drawTarget(original_image_->image, targetIn2d_);

    //          dir_msg_.header = image_msg->header;
    //          dir_msg_.vector.x = targetIn3d_.x;
    //          dir_msg_.vector.y = targetIn3d_.y;
    //          dir_msg_.vector.z = targetIn3d_.z;
    //          //fixme target_dir_pub_.publish(dir_msg_);
    //        } else {
    //            text = "direction: n/a";
    //        }
    //        //colorizer_.drawInfoText(input_bridge->image, text);
    //        TRACE_TO_ROS_INFO("%s", text.c_str());
            return true;
        }


        cv::Mat generateImageColoredBySampleColor(bool draw_target_positions = true) {
            cv::Mat image = original_image_->image.clone();
            colorizer_.colorPixelsBySampleColor(image, clusterizer_);
            if(draw_target_positions)
                drawTarget2dPositions(image);
            return image;
        }
        cv::Mat generateImageColoredByRandomColors(bool draw_target_positions = false) {
            cv::Mat image = original_image_->image.clone();
            colorizer_.colorPixelsByRandomColors(image, clusterizer_);
            if(draw_target_positions)
                drawTarget2dPositions(image);
            return image;
        }
        cv::Mat generateImageColoredByClusterStatus(bool draw_target_positions = true) {
            cv::Mat image = original_image_->image.clone();
            colorizer_.colorPixelsByStatusColor(image, clusterizer_);
            if(draw_target_positions)
                drawTarget2dPositions(image);
            return image;
        }


        void publishPreprocessedImage(image_transport::Publisher const& image_publisher) {
            if(image_publisher.getNumSubscribers() < 1)
                return;
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_image_->header, "bgr8", preprocessed_image_).toImageMsg();
            image_publisher.publish(img_msg);
        }
        void publishImageColoredBySampleColor(image_transport::Publisher const& image_publisher) {
            if(image_publisher.getNumSubscribers() < 1)
                return;
            cv::Mat image = generateImageColoredBySampleColor();
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_image_->header, "bgr8", image).toImageMsg();
            image_publisher.publish(img_msg);
        }
        void publishImageColoredByClusterStatus(image_transport::Publisher const& image_publisher) {
            if(image_publisher.getNumSubscribers() < 1)
                return;
            cv::Mat image = generateImageColoredByClusterStatus();
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_image_->header, "bgr8", image).toImageMsg();
            image_publisher.publish(img_msg);
        }

        void drawTarget2dPositions(cv::Mat& image) {
            for(std::size_t i = 0; i < clusterizer_.getNumOfCandidateClusters(); ++i) {
                cv::Point2d targetIn2d = clusterizer_.getObjectCenterInPixels(i);
                colorizer_.drawTarget(image, targetIn2d, clusterizer_.getObjectPpStatus(i));
            }
        }

        std::string generateTextSummary() const {
            std::size_t noc = clusterizer_.getNumOfCandidateClusters();
            if(noc == 0)
                return "...no good candidates found.";

            std::stringstream strstr;
            strstr << "got " << noc << " cluster candidates:" << std::endl;
            for(std::size_t i = 0; i < noc; ++i) {
                cv::Point2d targetIn2d = clusterizer_.getObjectCenterInPixels(i);
                obj_detector::ImageClusterizer::Cluster const& cluster = clusterizer_.getCandidateCluster(i);
                strstr << "  [" << i << "]: numOfAllPixels: " << cluster.num_of_points_
                       << ", numOfBorderPixels: " << cluster.num_of_boundary_points_
                       << ", status: " << (int)cluster.pp_status_
                       << ", hsv {" << (int)cluster.hsv_color_sample_.data_[0] << ", " << (int)cluster.hsv_color_sample_.data_[1] << ", " << (int)cluster.hsv_color_sample_.data_[2] << "}"
                       << ", 2dpos: {" << targetIn2d.x << ", " << targetIn2d.y << "}";
                if(got_cam_model_) {
                    cv::Point3d targetIn3d = mono_cam_model_.projectPixelTo3dRay(targetIn2d);
                    strstr << ", direction: {" << targetIn3d.x << ", " << targetIn3d.y << ", " << targetIn3d.z << "}";
                }
                strstr << std::endl;
            }
            return strstr.str();
        }

        std::string generateVerboseSummary() const {
            std::size_t noc = clusterizer_.getClusters().getTopClusterIndices().size();
            std::stringstream strstr;
            strstr << "total " << noc << " clusters:" << std::endl;
            for(std::size_t i = 0; i < noc; ++i) {
                clustering::ClusterIndex cluster_index = clusterizer_.getClusters().getTopClusterIndices()[i];
                obj_detector::ImageClusterizer::Cluster const& cluster = clusterizer_.getClusters().getTopCluster(cluster_index);
                strstr << "  [" << i << "]: numOfPixels: " << cluster.num_of_points_
                       << ", numOfBorderPixels: " << cluster.num_of_boundary_points_
                       << ", status: " << (int)cluster.pp_status_
                       << ", hsv {" << (int)cluster.hsv_color_sample_.data_[0] << ", " << (int)cluster.hsv_color_sample_.data_[1] << ", " << (int)cluster.hsv_color_sample_.data_[2] << "}"
                       << std::endl;
            }
            return strstr.str();
        }

        pixel_traits::ImageProcessParams params_;

        obj_detector::ImageClusterizer clusterizer_;
        obj_detector::Colorizer colorizer_;

        cv_bridge::CvImagePtr original_image_;
        cv::Mat preprocessed_image_;

        image_geometry::PinholeCameraModel mono_cam_model_;
        bool got_cam_model_;
        geometry_msgs::Vector3Stamped dir_msg_;
    };


    class Mono2dirNodeBase {
    protected:
      Nh& nh_;

      //boost::shared_ptr<image_transport::ImageTransport> it_;
      image_transport::ImageTransport it_;

      image_transport::Publisher sample_color_image_pub_;
      image_transport::Publisher cluster_status_image_pub_;

    //  tf::TransformListener tf_listener_;
    //  ros::Publisher target_dir_pub_;

      virtual FrameProcessor& getFrameProcessor() = 0;

    public:
      Mono2dirNodeBase(Nh& nh)
        : nh_(nh)
        , it_(nh_.nh_)
      {
        sample_color_image_pub_ = it_.advertise("/sample_color_image", 1);
        cluster_status_image_pub_ = it_.advertise("/cluster_status_image", 1);
    //    target_dir_pub_ = nh_.nh_.advertise<geometry_msgs::Vector3Stamped>(output_direction_topic_name, 100);
      }

      void monoImageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
        //TRACE_TO_ROS_INFO("TRACE: entered into Mono2dirNodeBase::monoImageCb(image_msg, info_msg)");
        //updateParams();
        if(!getFrameProcessor().processMonoImage(image_msg, info_msg)) {
            //TRACE_TO_ROS_INFO("TRACE: exitting Mono2dirNodeBase::monoImageCb(image_msg, info_msg) due to failure.");
            return;
        }

//        std::string verbose_text_info = getFrameProcessor().generateVerboseSummary();
//        TRACE_TO_ROS_INFO("TRACE(Mono2dirNodeBase): all clusters summary: %s", verbose_text_info.c_str());
        std::string text_info = getFrameProcessor().generateTextSummary();
        TRACE_TO_ROS_INFO("TRACE(Mono2dirNodeBase): candidates summary: %s", text_info.c_str());
        getFrameProcessor().publishImageColoredBySampleColor(sample_color_image_pub_);
        getFrameProcessor().publishImageColoredByClusterStatus(cluster_status_image_pub_);
        //fixme: publish direction to target
      }

      void monoImageCb(const sensor_msgs::ImageConstPtr& image_msg) {
          processImage(*image_msg);
      }

      void processImage(const sensor_msgs::Image& image_msg) {
          TRACE_TO_ROS_INFO("TRACE(Mono2dirNodeBase): entered into Mono2dirNodeBase::processImage(image_msg)");
          //updateParams();
          if(!getFrameProcessor().processMonoImage(image_msg))
              return;

          std::string verbose_text_info = getFrameProcessor().generateVerboseSummary();
          TRACE_TO_ROS_INFO("TRACE(Mono2dirNodeBase): all clusters summary: %s", verbose_text_info.c_str());
          std::string text_info = getFrameProcessor().generateTextSummary();
          TRACE_TO_ROS_INFO("TRACE(Mono2dirNodeBase): candidates summary: %s", text_info.c_str());
          getFrameProcessor().publishImageColoredBySampleColor(sample_color_image_pub_);
          getFrameProcessor().publishImageColoredByClusterStatus(cluster_status_image_pub_);
          TRACE_TO_ROS_INFO("TRACE(Mono2dirNodeBase): entered into Mono2dirNodeBase::processImage(image_msg) - done.");
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
        updateParams(getFrameProcessor().params_.color_cluster_params_);
      }

      void printParams() {
        printParams(getFrameProcessor().params_.color_cluster_params_);
      }

      void updateParams(pixel_traits::ColorClustersParams& params) {
        updateParam("clusterizer_color_distance_threshold", params.colorDistanceThreshold);
        updateParam("clusterizer_black_threshold",          params.black_threshold);
        updateParam("clusterizer_half_window_size",         params.halfWindowSize);
        updateParam("cluster_max_box_size",                 params.maxBoxSizeInBestCluster);
        updateParam("cluster_min_num_of_pixels",            params.minNumOfPixelsInBestCluster);
        updateParam("border_margin",                        params.border_margin);
        updateParam("shape_ratio",                          params.shape_ratio);
      }

      void printParams(pixel_traits::ColorClustersParams& params) {
        ROS_INFO("ColorClustersParams:");
        printParam("clusterizer_color_distance_threshold", params.colorDistanceThreshold);
        printParam("clusterizer_black_threshold",          params.black_threshold);
        printParam("clusterizer_half_window_size",         params.halfWindowSize);
        printParam("best_cluster_max_box_size",            params.maxBoxSizeInBestCluster);
        printParam("best_cluster_min_num_of_pixels",       params.minNumOfPixelsInBestCluster);
        printParam("border_margin",                        params.border_margin);
        printParam("shape_ratio",                          params.shape_ratio);
      }

    };

}

#endif
