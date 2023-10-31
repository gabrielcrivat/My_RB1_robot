#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include <cmath>
#include <ros/rate.h>
#include <ros/ros.h>

// global variables
double yaw_now = 0.0;
double yaw_init = 0.0;
ros::Publisher vel_pub;
ros::Subscriber odom_sub;
ros::ServiceServer my_service;

void odomCb(const nav_msgs::Odometry::ConstPtr &msg) {

  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  // current angle in rad

  yaw_now = atan2(2.0 * (y * x + w * z), w * w + x * x - y * y - z * z);
};

bool setRotCb(my_rb1_ros::Rotate::Request &req,
              my_rb1_ros::Rotate::Response &res) {

  ROS_INFO("Service Requested: /rotate_robot.");
  ROS_INFO("Degrees to rotate: %d", req.degrees);

  double yawRotRad = req.degrees * (M_PI / 180);
  yaw_init = yaw_now;
  double fin_yaw = yaw_init + yawRotRad; // goal_yaw in rad
  double kP = 0.5;

  while (fabs(fin_yaw - yaw_init) > 0.017) {
    ros::spinOnce();
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z =
        kP * (fin_yaw - yaw_init); // proportional controller. takes into
                                   // account direction and modulus omega
    vel_pub.publish(vel_msg);
    yaw_init = yaw_now;
    /*ROS_INFO("fin_yaw: %lf", fin_yaw);
    ROS_INFO("yaw_now: %lf", yaw_now);
    ROS_INFO("yaw_init: %lf", yaw_init);*/
  };

  geometry_msgs::Twist twist_cmd;
  twist_cmd.angular.z = 0; // stop the robot
  twist_cmd.linear.x = 0;
  twist_cmd.linear.y = 0;
  vel_pub.publish(twist_cmd);

  res.result = "Rotation Completed";
  ROS_INFO("Service Complete: /rotate_robot");
  return true;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "rotate_robot_node");

  ros::NodeHandle nh;

  while (ros::ok()) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    odom_sub = nh.subscribe("/odom", 10, odomCb);

    my_service = nh.advertiseService("/rotate_robot", setRotCb);

    ROS_INFO("Service is Ready: /rotate_robot");

    ros::spin();
  };
  return 0;
};
