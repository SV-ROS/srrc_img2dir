#include "stereo2dir.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo2dir");
  img2dir::Nh nh;
  img2dir::Stereo2dir ic(nh);
  ros::spin();
  return 0;
}
