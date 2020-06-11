#include "dvs_integrator_conv/integrator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_integrator_conv");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_integrator_conv::Integrator integrator(nh, nh_private);

  ros::spin();

  return 0;
}
