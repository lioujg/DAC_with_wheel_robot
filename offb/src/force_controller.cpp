#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>

geometry_msgs::Wrench robot_cmd;
sensor_msgs::Imu payload_imu;

void payload_orientation_cb(const sensor_msgs::Imu::ConstPtr &msg){
  payload_imu = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_controller");
  ros::NodeHandle nh;

  ros::Subscriber payload_imu_sub = nh.subscribe<sensor_msgs::Imu>("/payload/IMU",4,payload_orientation_cb);
  ros::Publisher robot_controller_pub = nh.advertise<geometry_msgs::Wrench>("robot_wrench",4);
  ros::Rate loop_rate(20);

  while(ros::ok()){

    // robot_cmd.force.z = 0.5 / 2 * 9.81;
    robot_cmd.torque.z = 3;

  	robot_controller_pub.publish(robot_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}