#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "utils.h"

class TestImagePublisher {
public:
    TestImagePublisher(img2dir::Nh& nh)
        : nh_(nh)
        , it_(nh.nh_)
    {
        std::string image_src_topic_name = "/turret_stereo/left/image_raw";
        updateParam("src_topic", image_src_topic_name);
        image_pub_ = nh_.nh_.advertise<sensor_msgs::Image>(image_src_topic_name, 1);

        std::string image_result_topic_name = "/cluster_status_image";
        updateParam("result_topic", image_result_topic_name);
        std::string input_image_transfer_hint = "compressed";
        updateParam("input_image_transfer_hint", input_image_transfer_hint);
        result_image_sub_ = it_.subscribe(image_result_topic_name, 1, &TestImagePublisher::resultImageCb, this, image_transport::TransportHints(input_image_transfer_hint));

        std::string image_trg_folder_name = "";
        updateParam("output_folder", image_trg_folder_name);
        std::string params_file_name = "/home/dd/catkin_ws/src/nasa_srrc/srrc_img2dir/config/mono_cam_params.yaml";
        updateParam("params_file_name", params_file_name);
        initOutputFolder(image_trg_folder_name, params_file_name);
    }

    void publishImagesFromFolder(std::string const& image_folder_path) {
        input_image_itr_ = boost::filesystem::directory_iterator(image_folder_path);
        publishNextImage(true);
    }

    void publishImage(std::string const& image_file_name) {
        ROS_INFO("TRACE(test_image_pub): publishing image %s", image_file_name.c_str());
        if(!boost::filesystem::exists(image_file_name)) {
            ROS_ERROR("ERROR(test_image_pub): file %s does not exist.", image_file_name.c_str());
            return;
        }
        src_image_.image = cv::imread(image_file_name, CV_LOAD_IMAGE_COLOR);
//        cv::imshow("original image", src_image_.image);
//        cv::waitKey();
//        ROS_INFO("TRACE(test_image_pub): entered into publishImage(image_msg)-1");
        src_image_.encoding = "bgr8";
        sensor_msgs::Image ros_image;
        src_image_.toImageMsg(ros_image);
        last_published_image_file_name_ = boost::filesystem::path(image_file_name).filename().string();
        image_pub_.publish(ros_image);
        ROS_INFO("TRACE(test_image_pub): publishing image %s - done.", image_file_name.c_str());
    }

private:
    void resultImageCb(const sensor_msgs::ImageConstPtr& image_msg) {
        ROS_INFO("TRACE(test_image_pub): got result image for %s", last_published_image_file_name_.c_str());
        //: assumes the mono2dir node publishes only images obtained from processing images published by this node and nobody else publish to the source and result topics!
        cv_bridge::CvImagePtr result_image;
        try {
          result_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (...){
          ROS_ERROR("[draw_frames] Failed to convert image");
          return;
        }
        int rows = std::max(src_image_.image.size().height, result_image->image.size().height);
        int cols = src_image_.image.size().width + result_image->image.size().width + 2;
        cv::Mat stitch = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0));
        src_image_.image.copyTo(stitch.colRange(0, src_image_.image.size().width).rowRange(0, src_image_.image.size().height));
        result_image->image.copyTo(stitch.colRange(src_image_.image.size().width + 1, src_image_.image.size().width + result_image->image.size().width + 1)
                                   .rowRange(0, result_image->image.size().height));
        if(!output_folder_.empty() && !last_published_image_file_name_.empty()) {
            boost::filesystem::path image_output_path = output_folder_ / last_published_image_file_name_;
            cv::imwrite(image_output_path.string(), stitch);
            ROS_INFO("TRACE(test_image_pub): saved output image %s", image_output_path.c_str());
        }
        ROS_INFO("TRACE(test_image_pub): got result image for %s - done.", last_published_image_file_name_.c_str());
        cv::imshow("clustered image", stitch);
        cv::waitKey();
        publishNextImage();
    }

private:
    void initOutputFolder(std::string const& root_output_folder, std::string const& params_file_name) {
        if(root_output_folder.empty())
            return;
        output_folder_ = root_output_folder;
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        std::string generated_folder_name = boost::posix_time::to_iso_extended_string(now);
        std::string chars_to_replace = " ,:";
        BOOST_FOREACH(char& ch, generated_folder_name) {
            if(chars_to_replace.find(ch) != std::string::npos)
                ch = '_';
        }
        output_folder_ /= generated_folder_name;
        boost::filesystem::create_directories(output_folder_);
        if(boost::filesystem::exists(params_file_name)) {
            boost::system::error_code ec;
            boost::filesystem::copy_file(params_file_name, output_folder_, ec);
            if(!ec) {
                std::string err_msg = ec.message();
                ROS_WARN("TRACE(test_image_pub): Could not copy file %s. Reason: %s", params_file_name.c_str(), err_msg.c_str());
            }
        }
        ROS_INFO("TRACE(test_image_pub): see for output images in %s", output_folder_.c_str());
    }

    void publishNextImage(bool first = false) {
        boost::filesystem::directory_iterator end_itr;
        if(input_image_itr_ == end_itr) {
            ROS_INFO("TRACE(test_image_pub): Done with publishing images.");
            return;
        }
        if(!first)
            ++input_image_itr_;
        while(input_image_itr_ != end_itr
              && !boost::filesystem::is_regular_file(input_image_itr_->path())
              && input_image_itr_->path().string().size() < 4
              && input_image_itr_->path().string().substr(input_image_itr_->path().string().size() - 4, 4) != ".jpg"
              )
            ++input_image_itr_;
        if(input_image_itr_ == end_itr) {
            ROS_INFO("TRACE(test_image_pub): Done with publishing images.");
            return;
        }
        publishImage(input_image_itr_->path().string());
    }

    template<typename t_Param>
    static void printParam(std::string const& param_name, t_Param& param, std::string const& prefix = " * ") {
        img2dir::Nh::printParam(param_name, param, prefix);
    }

    template<typename t_Param>
    void updateParam(std::string const& param_name, t_Param& param) {
        nh_.updateParam(param_name, param);
    }

private:
    img2dir::Nh& nh_;
    image_transport::ImageTransport it_;
    ros::Publisher image_pub_;
    image_transport::Subscriber result_image_sub_;
    boost::filesystem::path output_folder_;

    boost::filesystem::directory_iterator input_image_itr_;
    std::string last_published_image_file_name_;
    cv_bridge::CvImage src_image_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_image_pub");
  img2dir::Nh nh;
  TestImagePublisher node(nh);

  std::string image_file_name = "";
  nh.updateParam("src_image_file", image_file_name);
  std::string image_src_folder_name = "~/img2dir_sources";
  nh.updateParam("src_image_src_folder", image_src_folder_name);
  nh.printParam("src_image_file", image_file_name);
  nh.printParam("src_image_src_folder", image_src_folder_name);

  ros::Rate sleep_rate(ros::Duration(1, 0));
  sleep_rate.sleep(); //fixme: need a way to ensure the mono2dir node started
  if(!image_file_name.empty())
      node.publishImage(image_file_name);
  else
      node.publishImagesFromFolder(image_src_folder_name);

  ros::spin();
  return 0;
}

//{
//  std::string image_trg_folder_name;
//  nh.param("trg_folder", image_trg_folder_name, "");

//  std::string image_topic_name;
//  nh.param("topic", image_topic_name, "/turret_stereo/left/image_raw");

//  cv_bridge::CvImage cv_image;
//  cv_image.image = cv::imread(image_file_name,CV_LOAD_IMAGE_COLOR);
//  cv_image.encoding = "bgr8";
//  sensor_msgs::Image ros_image;
//  cv_image.toImageMsg(ros_image);

//  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(image_topic_name, 1);
//  ros::Rate loop_rate(5);

//  while (nh.ok()) {
//    pub.publish(ros_image);
//    loop_rate.sleep();
//  }
//}
