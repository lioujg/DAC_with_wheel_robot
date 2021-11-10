#include "ros/ros.h"
// #include "forces_torques/"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <cstdlib>
#include <geometry_msgs/Wrench.h>
#include <string>
#include "std_msgs/String.h"

geometry_msgs::Wrench robot_cmd;

void robot_wrench_cb(const geometry_msgs::Wrench::ConstPtr &msg){
  robot_cmd = *msg;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_control_client");
  ros::NodeHandle n;
  std::string robot_number;
  n.getParam("robot_number", robot_number);

  ros::Subscriber robot_cmd_sub = n.subscribe<geometry_msgs::Wrench>("robot_wrench",4,robot_wrench_cb);

  ros::ServiceClient wrenchClient = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
  gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

  // ros::Time time_temp(0, 0);
  // ros::Duration duration_temp(0, 1000000);
  // apply_wrench_req.start_time = time_temp;
  // apply_wrench_req.duration = duration_temp;

  apply_wrench_req.start_time = ros::Time(0);
  apply_wrench_req.duration = ros::Duration(-1);
  apply_wrench_req.body_name = "payload::payload_link_" + robot_number;

  bool call_service;
  ros::Rate loop_rate(20);

  while(ros::ok()){
    apply_wrench_req.wrench.force.x = robot_cmd.force.x;
    apply_wrench_req.wrench.force.y = robot_cmd.force.y;
    apply_wrench_req.wrench.force.z = robot_cmd.force.z;

    apply_wrench_req.wrench.torque.z = robot_cmd.torque.z;
    apply_wrench_req.wrench.torque.z = robot_cmd.torque.z;
    apply_wrench_req.wrench.torque.z = robot_cmd.torque.z;

  	call_service = wrenchClient.call(apply_wrench_req, apply_wrench_resp);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}