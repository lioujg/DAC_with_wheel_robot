#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>


Eigen::Matrix3d R;

sensor_msgs::Imu payload_imu;
void payload_orientation_cb(const sensor_msgs::Imu::ConstPtr &msg){
  payload_imu = *msg;
  Eigen::Quaterniond q;
  float w,x,y,z;
  q.x() = payload_imu.orientation.x;
  q.y() = payload_imu.orientation.y;
  q.z() = payload_imu.orientation.z;
  q.w() = payload_imu.orientation.w;

  R = q.normalized().toRotationMatrix();
  // std::cout << "R=" << std::endl << R << std::endl;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint reference_input;
void reference_cb(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg){
  reference_input = *msg;
  // reference_input.velocities[0].linear.x;
  // reference_input.velocities[0].linear.y;
  // reference_input.velocities[0].linear.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_controller");
  ros::NodeHandle nh;

  ros::Subscriber payload_imu_sub = nh.subscribe<sensor_msgs::Imu>("/payload/IMU",4,payload_orientation_cb);
  ros::Subscriber reference_input_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/payload/desired_trajectory",4,reference_cb);

  ros::Publisher robot_controller_pub = nh.advertise<geometry_msgs::Wrench>("robot_wrench",4);
  ros::Rate loop_rate(20);

  while(ros::ok()){

    geometry_msgs::Wrench robot_cmd;
    robot_cmd.force.z = 0.5 / 2 * 9.81;
    // robot_cmd.torque.z = 3;

  	robot_controller_pub.publish(robot_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}