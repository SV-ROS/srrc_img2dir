#include "mono2dir.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono2dir");
  img2dir::Nh nh;
  img2dir::Mono2dir ic(nh);
  ros::spin();
  return 0;
}
