// mono2dir.h

#ifndef MONO2DIR_H
#define MONO2DIR_H

#include "ImageClusterizer.h"

namespace img2dir {


class Mono2dir : public Mono2dirNodeBase {
  image_transport::CameraSubscriber mono_camera_sub_;
  image_transport::Subscriber mono_image_sub_;

  FrameProcessor frame_processor_;

  virtual FrameProcessor& getFrameProcessor() { return frame_processor_; }

public:
  Mono2dir(Nh& nh)
    : Mono2dirNodeBase(nh)
  {
    updateParams();
    printParams();

    std::string input_image_transfer_hint     = "compressed";
    std::string input_image_topic             = "/turret_stereo/left/image_raw";
    std::string output_direction_topic_name   = "target_dir";
    updateParam("input_image_transfer_hint",   input_image_transfer_hint);
    updateParam("input_image_topic",           input_image_topic);
    updateParam("output_direction_topic_name", output_direction_topic_name);

    printParam("input_image_topic",            input_image_topic);
    printParam("input_image_transfer_hint",    input_image_transfer_hint);
    printParam("output_direction_topic_name",  output_direction_topic_name);

    bool use_camera_info = false;//fixme!
    updateParam("camera_use_camera_info", use_camera_info);
    printParam ("camera_use_camera_info", use_camera_info);

    Mono2dirNodeBase* base_this = this;
    if(use_camera_info) {
        mono_camera_sub_ = it_.subscribeCamera(input_image_topic, 1, &Mono2dirNodeBase::monoImageCb, base_this, image_transport::TransportHints(input_image_transfer_hint));
    } else {
        mono_image_sub_ = it_.subscribe(input_image_topic, 1, &Mono2dirNodeBase::monoImageCb, base_this, image_transport::TransportHints(input_image_transfer_hint));
    }
  }
};

}
#endif
