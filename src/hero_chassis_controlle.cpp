#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>

float wheel_track = 0.4;
float wheel_base = 0.4;
float rx = wheel_track / 2;
float ry = wheel_base / 2;

namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override
  {
    left_front_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
    right_front_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
    left_back_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    right_back_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

    // Load PID parameters from the parameter server
    left_front_pid_.setGains(1.0, 0.0, 0.0, 1.0, -1.0);
    right_front_pid_.setGains(1.0, 0.0, 0.0, 1.0, -1.0);
    left_back_pid_.setGains(1.0, 0.0, 0.0, 1.0, -1.0);
    right_back_pid_.setGains(1.0, 0.0, 0.0, 1.0, -1.0);

    // Subscribe to the cmd_vel topic to get the desired velocities
    cmd_vel_subscriber_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period) override
  {
    // Compute control effort for each wheel
    double left_front_effort =
        left_front_pid_.computeCommand(desired_left_front_velocity_ - left_front_joint_.getVelocity(), period);
    double right_front_effort =
        right_front_pid_.computeCommand(desired_right_front_velocity_ - right_front_joint_.getVelocity(), period);
    double left_back_effort =
        left_back_pid_.computeCommand(desired_left_back_velocity_ - left_back_joint_.getVelocity(), period);
    double right_back_effort =
        right_back_pid_.computeCommand(desired_right_back_velocity_ - right_back_joint_.getVelocity(), period);

    // Set the control effort to the joints
    left_front_joint_.setCommand(left_front_effort);
    right_front_joint_.setCommand(right_front_effort);
    left_back_joint_.setCommand(left_back_effort);
    right_back_joint_.setCommand(right_back_effort);
  }

private:
  void cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
  {
    // 逆运动学解算，参考https://www.robotsfan.com/posts/b6e9d4e.html，cmd_vel.angular前系数由轮距和轮距决定
    desired_left_front_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
    desired_right_front_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
    desired_left_back_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
    desired_right_back_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
  }

  hardware_interface::JointHandle left_front_joint_, right_front_joint_, left_back_joint_, right_back_joint_;
  control_toolbox::Pid left_front_pid_, right_front_pid_, left_back_pid_, right_back_pid_;
  ros::Subscriber cmd_vel_subscriber_;

  double desired_left_front_velocity_{ 0.0 };
  double desired_right_front_velocity_{ 0.0 };
  double desired_left_back_velocity_{ 0.0 };
  double desired_right_back_velocity_{ 0.0 };
};

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller