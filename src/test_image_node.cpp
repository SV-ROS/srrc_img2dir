#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "ImageClusterizer.h"

namespace img2dir {


class TestImageProcessor : public Mono2dirNodeBase {
  FrameProcessor frame_processor_;

  virtual FrameProcessor& getFrameProcessor() { return frame_processor_; }

public:
  typedef Mono2dirNodeBase Base;

  TestImageProcessor(Nh& nh) : Base(nh) {
    updateParams();
    printParams();

    std::string image_trg_folder_name = "";
    updateParam("output_folder", image_trg_folder_name);
    std::string params_file_name = "/home/dd/catkin_ws/src/nasa_srrc/srrc_img2dir/config/mono_cam_params.yaml";
    updateParam("params_file_name", params_file_name);
    initOutputFolder(image_trg_folder_name, params_file_name);
  }

  void processImagesFromFolder(std::string const& image_folder_path) {
      ROS_INFO("TRACE(test_image_process): processing folder %s", image_folder_path.c_str());
      boost::filesystem::directory_iterator end_itr;
      boost::filesystem::directory_iterator input_image_itr = findNextImage(boost::filesystem::directory_iterator(image_folder_path));
      while(input_image_itr != end_itr) {
          processImage(input_image_itr->path().string());
          findNextImage(++input_image_itr);
      }
      ROS_INFO("TRACE(test_image_process): Done with processing images.");
  }

  void processImage(std::string const& image_file_name) {
      ROS_INFO("TRACE(test_image_process): processing image %s", image_file_name.c_str());
      if(!boost::filesystem::exists(image_file_name)) {
          ROS_ERROR("ERROR(test_image_process): file %s does not exist.", image_file_name.c_str());
          return;
      }
      src_image_.image = cv::imread(image_file_name, CV_LOAD_IMAGE_COLOR);
//        cv::imshow("original image", src_image_.image);
//        cv::waitKey();
//        ROS_INFO("TRACE(test_image_process): entered into processImage(image_msg)-1");
      src_image_.encoding = "bgr8";
      sensor_msgs::Image ros_image;
      src_image_.toImageMsg(ros_image);
      Base::processImage(ros_image);
      makeResultImage(image_file_name);
      ROS_INFO("TRACE(test_image_process): processing image %s - done.", image_file_name.c_str());
  }

private:
  void makeResultImage(std::string const& image_file_name) {
      ROS_INFO("TRACE(test_image_process): in makeResultImage(%s)", image_file_name.c_str());
      int rows = src_image_.image.size().height;
      int cols = src_image_.image.size().width;
      cv::Mat stitch = cv::Mat(2 * rows + 2, 2 * cols + 2, CV_8UC3, cv::Scalar(0));

      //: top-left: original image:
      src_image_.image.copyTo(stitch.colRange(0, cols).rowRange(0, rows));

      //: top-right: clusters by (warped) sample color:
      {
          cv::Mat image = getFrameProcessor().generateImageColoredBySampleColor();
          image.copyTo(stitch.colRange(cols + 1, 2 * cols + 1).rowRange(0, rows));
      }

//      //: bottom-left: clusters by random color:
//      {
//          cv::Mat image = getFrameProcessor().generateImageColoredByRandomColors();
//          image.copyTo(stitch.colRange(0, cols).rowRange(rows + 1, 2 * rows + 1));
//      }

      //: bottom-left: prepocessed original image:
      {
          cv::Mat const& image = getFrameProcessor().preprocessed_image_;
          image.copyTo(stitch.colRange(0, cols).rowRange(rows + 1, 2 * rows + 1));
      }

      //: bottom-right: clusters by "status":
      {
          cv::Mat image = getFrameProcessor().generateImageColoredByClusterStatus();
          image.copyTo(stitch.colRange(cols + 1, 2 * cols + 1).rowRange(rows + 1, 2 * rows + 1));
      }

      if(!output_folder_.empty()) {
          boost::filesystem::path image_output_path = output_folder_ / boost::filesystem::path(image_file_name).filename();
          cv::imwrite(image_output_path.string(), stitch);
          ROS_INFO("TRACE(test_image_process): saved output image %s", image_output_path.c_str());
      }
      ROS_INFO("TRACE(test_image_process): in makeResultImage(%s) - done.", image_file_name.c_str());
//      cv::imshow("clustered image", stitch);
//      cv::waitKey();
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

  boost::filesystem::directory_iterator findNextImage(boost::filesystem::directory_iterator input_image_itr) {
      boost::filesystem::directory_iterator end_itr;
      while(input_image_itr != end_itr
            && !boost::filesystem::is_regular_file(input_image_itr->path())
            && input_image_itr->path().string().size() < 4
            && input_image_itr->path().string().substr(input_image_itr->path().string().size() - 4, 4) != ".jpg"
            )
          ++input_image_itr;
      return input_image_itr;
  }

private:
  boost::filesystem::path output_folder_;
  cv_bridge::CvImage src_image_;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_image_processor");
  img2dir::Nh nh;
  img2dir::TestImageProcessor node(nh);

  std::string image_file_name = "";
  nh.updateParam("src_image_file", image_file_name);
  std::string image_src_folder_name = "/home/dd/img2dir_sources";
  nh.updateParam("src_image_src_folder", image_src_folder_name);
  nh.printParam("src_image_file", image_file_name);
  nh.printParam("src_image_src_folder", image_src_folder_name);

  if(!image_file_name.empty())
      node.processImage(image_file_name);
  else
      node.processImagesFromFolder(image_src_folder_name);
  return 0;
}
