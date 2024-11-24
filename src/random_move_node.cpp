//
// Created by hyd on 24-11-24.
//
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

  ros::Rate rate(1); // 1 Hz

  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = (std::rand() % 200 - 100) / 100.0;  // Random linear velocity between -1 and 1
    msg.linear.y = (std::rand() % 200 - 100) / 100.0;  // Random linear velocity between -1 and 1
    msg.linear.z = 0.0;
    msg.angular.z = (std::rand() % 200 - 100) / 100.0; // Random angular velocity between -1 and 1

    cmd_vel_pub.publish(msg);

    rate.sleep();
  }

  return 0;
}