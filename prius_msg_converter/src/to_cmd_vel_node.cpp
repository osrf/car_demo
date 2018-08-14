#include <to_cmd_vel.hpp>

#include <ros/ros.h>

namespace pmc = prius::msg_converter;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "prius_msg_to_cmd_vel_node");
  pmc::ToCmdVel pmc_tcv;
  pmc_tcv.CtrlUpdate();
}
