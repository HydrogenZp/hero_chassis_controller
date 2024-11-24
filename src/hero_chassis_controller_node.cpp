#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "hero_chassis_controller/hero_chassis_controller.h"
#include "control_toolbox/pid.h"

namespace hero_chassis_controller
{
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // 初始化关节Handle
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  return true;
};





PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller