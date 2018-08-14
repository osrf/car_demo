#ifndef TO_CMD_VEL_HPP_INCLUDED
#define TO_CMD_VEL_HPP_INCLUDED

// usual header for ROS
#include <ros/ros.h>

// messages to use
#include <prius_msgs/Control.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// for std lib
#include <iostream>
#include <mutex>


namespace prius {
  namespace msg_converter {
    class ToCmdVel {
    public:
      ToCmdVel();
      ~ToCmdVel();
      void CtrlUpdate();
    private:
      void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
      void CurStateCallback(const nav_msgs::Odometry::ConstPtr &msg);

      // node handler
      ros::NodeHandle nh_;
      
      // subscriber
      ros::Subscriber cmd_vel_sub_;
      ros::Subscriber current_state_sub_;
      // publisher
      ros::Publisher control_msg_pub_;

      // tmp_messages
      geometry_msgs::Twist cmd_vel_;
      nav_msgs::Odometry current_state_;

      // subscribe message mutex
      std::mutex cmd_vel_mtx_;
      std::mutex current_state_mtx_;
      
      // parameters
      double wheel_base_;
      double max_steer_angle_;
      double max_vel_;
      double kp_throttle_;
      double ki_throttle_;
      double kp_brake_;
      double ki_brake_;
      double pub_rate_;
    };
  }
}

#endif
