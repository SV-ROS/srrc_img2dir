// camera_pitch_yaw.h

#ifndef CAMERA_PITCH_YAW_H
#define CAMERA_PITCH_YAW_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#include "utils.h"

namespace img2dir
{

class CameraPitchYaw
{
  Nh& nh_;
  ros::Publisher pitch_pub_;
  ros::Publisher yaw_pub_;

  ros::Subscriber camera_joint_states_sub_;

  PitchYaw camera_pitch_yaw_;
  double pitch_velocity_;
  double yaw_velocity_;
  bool is_camera_moving_;

  //tf::TransformListener tf_listener_;

public:
  CameraPitchYaw(Nh& nh)
    : nh_(nh)
    , camera_pitch_yaw_()
    , pitch_velocity_(0)
    , yaw_velocity_(0)
    , is_camera_moving_(false)
  {
      pitch_pub_ = nh_.nh_.advertise<std_msgs::Float64>("/camera_pitch_joint/command", 1);
      yaw_pub_ = nh_.nh_.advertise<std_msgs::Float64>("/camera_yaw_joint/command", 1);
      camera_joint_states_sub_ = nh_.nh_.subscribe("/camera_joint_states", 10, &CameraPitchYaw::cameraJointStatesCb, this);

      //: 'Give it some time to start producing something'
//      ttf_listener_f.waitForTransform('/base_footprint', '/camera_optical_frame', ros::Time(0), ros::Duration(2.0));
  }

  void rotateCameraTo(double pitch_in_rad, double yaw_in_rad) {
      std_msgs::Float64 msg;
      msg.data = pitch_in_rad;
      pitch_pub_.publish(msg);
      msg.data = yaw_in_rad;
      yaw_pub_.publish(msg);
  }

  void rotateCameraBy(double delta_pitch_in_rad, double delta_yaw_in_rad) {
      rotateCameraTo(camera_pitch_yaw_.pitch_ + delta_pitch_in_rad, camera_pitch_yaw_.yaw_ + delta_yaw_in_rad);
  }

  void centerCameraAt(geometry_msgs::Vector3 const& dir_vector_in_cam_system) {
      PitchYaw py(dir_vector_in_cam_system);
      rotateCameraBy(py.pitch_, py.yaw_);
  }

private:
  void cameraJointStatesCb(const sensor_msgs::JointStateConstPtr& msg) {
      for(int i = 0; i < msg->name.size(); ++i) {
          if(msg->name[i] == "yaw_joint") {
              ROS_INFO("camera yaw: old=%f, new=%f", camera_pitch_yaw_.yaw_, msg->position[i]);
              camera_pitch_yaw_.yaw_ = msg->position[i];
              yaw_velocity_ = msg->velocity[i];
              is_camera_moving_ = is_camera_moving_ || (yaw_velocity_ == 0);
          } else if(msg->name[i] == "pitch_joint") {
              ROS_INFO("camera pitch: old=%f, new=%f", camera_pitch_yaw_.pitch_, msg->position[i]);
              camera_pitch_yaw_.pitch_ = msg->position[i];
              yaw_velocity_ = msg->velocity[i];
              is_camera_moving_ = is_camera_moving_ || (pitch_velocity_ == 0);
          }
      }
  }

};

}
#endif
