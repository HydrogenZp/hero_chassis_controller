#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_move_node");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  std::srand(std::time(0));

  ros::Rate rate(0.7);  //  Update at 0.7 Hz

  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = static_cast<double>((std::rand() % 200 - 100)) / 40;  // Random linear velocities
    msg.linear.y = static_cast<double>((std::rand() % 200 - 100)) / 40;  // Random linear velocities
    msg.linear.z = 0.0;
    msg.angular.z = static_cast<double>((std::rand() % 200 - 100)) / 50;

    cmd_vel_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}