// utils.h

#ifndef IMG2DIR_UTILS_H
#define IMG2DIR_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

namespace img2dir {

struct Nh {
  ros::NodeHandle private_nh_;
  ros::NodeHandle nh_;

  Nh()
    : private_nh_("~")
    , nh_()
  {
  }

  template<typename t_Param>
  static void printParam(std::string const& param_name, t_Param& param, std::string const& prefix = " (*) ") {
      std::stringstream strstr;
      strstr << prefix << param_name << ": " << param;
      std::string text = strstr.str();
      ROS_INFO("%s", text.c_str());
  }

  template<typename t_Param>
  static void updateParam(ros::NodeHandle& nh, std::string const& param_name, t_Param& param) {
    t_Param old = param;
    nh.param(param_name, param, old);
    if(param != old)
      printParam(param_name, param, "updated ");
  }

  template<typename t_Param>
  void updateParam(std::string const& param_name, t_Param& param) {
    updateParam(private_nh_, param_name, param);
  }

};

struct PitchYaw {
    double pitch_;
    double yaw_;

    PitchYaw() : pitch_(0), yaw_(0) {}
    PitchYaw(double pitch, double yaw) : pitch_(pitch), yaw_(yaw) {}
};

inline PitchYaw getPitchYawFromDirInCameraSystem(double dir_x, double dir_y, double dir_z) {
    double pitch =  std::atan2(dir_y, dir_z);
    double yaw   = -std::atan2(dir_x, dir_z);
    return PitchYaw(pitch, yaw);
}

}

#endif
