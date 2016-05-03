#include "img2dir.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img2dir");
  img2dir::Nh nh;
  img2dir::Img2dir ic(nh);
  ros::spin();
  return 0;
}
