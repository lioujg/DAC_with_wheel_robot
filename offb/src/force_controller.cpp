#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

Eigen::Matrix3d R;
Eigen::Matrix3d R_d;
geometry_msgs::Twist payload_velocity;
geometry_msgs::Twist payload_reference_velocity;
geometry_msgs::Point payload_position;
geometry_msgs::Point payload_reference_position;


void payload_orientation_cb(const sensor_msgs::Imu::ConstPtr &msg){
  sensor_msgs::Imu payload_imu;
  payload_imu = *msg;
  Eigen::Quaterniond q;
  q.x() = payload_imu.orientation.x;
  q.y() = payload_imu.orientation.y;
  q.z() = payload_imu.orientation.z;
  q.w() = payload_imu.orientation.w;

  R = q.normalized().toRotationMatrix();
  // std::cout << "R=" << std::endl << R << std::endl;

  payload_velocity.angular = payload_imu.angular_velocity;
}

void reference_cb(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg){
  trajectory_msgs::MultiDOFJointTrajectoryPoint reference_input;
  reference_input = *msg;
  payload_reference_velocity.linear = reference_input.velocities[0].linear;
  payload_reference_velocity.angular.x = 0;
  payload_reference_velocity.angular.y = 0;
  payload_reference_velocity.angular.z = 0;
  payload_reference_position.x = reference_input.transforms[0].translation.x;
  payload_reference_position.y = reference_input.transforms[0].translation.y;
  payload_reference_position.z = reference_input.transforms[0].translation.z;
}

void payload_odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
  nav_msgs::Odometry payload_odom;
  payload_odom = *msg;
  payload_position = payload_odom.pose.pose.position;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_controller");
  ros::NodeHandle nh;

  ros::Subscriber payload_imu_sub = nh.subscribe<sensor_msgs::Imu>("/payload/IMU",4,payload_orientation_cb);
  ros::Subscriber reference_input_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/payload/desired_trajectory",4,reference_cb);
  ros::Subscriber payload_odom_sub = nh.subscribe<nav_msgs::Odometry>("/payload/payload_position",4,payload_odom_cb);

  ros::Publisher robot_controller_pub = nh.advertise<geometry_msgs::Wrench>("robot_wrench",4);
  ros::Rate loop_rate(20);

  while(ros::ok()){
    // compute error signals
    

    geometry_msgs::Wrench robot_cmd;
    robot_cmd.force.z = 0.5 / 2 * 9.81;
    // robot_cmd.torque.z = 3;

  	robot_controller_pub.publish(robot_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}