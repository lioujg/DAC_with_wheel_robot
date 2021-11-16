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

Eigen::Vector3d payload_linear_velocity;
Eigen::Vector3d payload_angular_velocity;
Eigen::Vector3d payload_reference_linear_velocity;
Eigen::Vector3d payload_reference_angular_velocity;
Eigen::Vector3d payload_position;
Eigen::Vector3d payload_reference_position;
Eigen::Vector3d payload_reference_linear_acceleration;
Eigen::Vector3d payload_reference_angular_acceleration;


double lambda;


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

  payload_angular_velocity(0) = payload_imu.angular_velocity.x;
  payload_angular_velocity(1) = payload_imu.angular_velocity.y;
  payload_angular_velocity(2) = payload_imu.angular_velocity.z;
}

void reference_cb(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg){
  trajectory_msgs::MultiDOFJointTrajectoryPoint reference_input;
  reference_input = *msg;
  payload_reference_linear_velocity(0) = reference_input.velocities[0].linear.x;
  payload_reference_linear_velocity(1) = reference_input.velocities[0].linear.y;
  payload_reference_linear_velocity(2) = reference_input.velocities[0].linear.z;
  payload_reference_angular_velocity(0) = 0;
  payload_reference_angular_velocity(1) = 0;
  payload_reference_angular_velocity(2) = 0;
  payload_reference_position(0) = reference_input.transforms[0].translation.x;
  payload_reference_position(1) = reference_input.transforms[0].translation.y;
  payload_reference_position(2) = reference_input.transforms[0].translation.z;
  payload_reference_linear_acceleration(0) = reference_input.accelerations[0].linear.x;
  payload_reference_linear_acceleration(1) = reference_input.accelerations[0].linear.y;
  payload_reference_linear_acceleration(2) = reference_input.accelerations[0].linear.z;
  payload_reference_angular_acceleration(0) = 0;
  payload_reference_angular_acceleration(1) = 0;
  payload_reference_angular_acceleration(2) = 0;
}

void payload_odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
  nav_msgs::Odometry payload_odom;
  payload_odom = *msg;
  payload_position(0) = payload_odom.pose.pose.position.x;
  payload_position(1) = payload_odom.pose.pose.position.y;
  payload_position(2) = payload_odom.pose.pose.position.z;
  payload_linear_velocity(0) = payload_odom.twist.twist.linear.x;
  payload_linear_velocity(1) = payload_odom.twist.twist.linear.y;
  payload_linear_velocity(2) = payload_odom.twist.twist.linear.z;
}

void initialized_params(){
  // initialize the desired rotation matrix
  R_d << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  double lambda = 1.5;
}

Eigen::Vector3d vee_map(Eigen::Matrix3d Matrix){
  Eigen::Vector3d vee;
  vee(0) = Matrix(2, 1);
  vee(1) = Matrix(0, 2);
  vee(2) = Matrix(1, 0);
  return vee;
}

Eigen::Matrix3d hat_map(Eigen::Vector3d Vector){
  Eigen::Matrix3d hat;
  hat(0, 0) = 0;
  hat(0, 1) = -Vector(2);
  hat(0, 2) = Vector(1);
  hat(1, 0) = Vector(2);
  hat(1, 1) = 0;
  hat(1, 2) = -Vector(0);
  hat(2, 0) = -Vector(1);
  hat(2, 1) = Vector(0);
  hat(2, 2) = 0;
  return hat;
}

Eigen::Matrix3d Pa(Eigen::Matrix3d Matrix){
  Eigen::Matrix3d Matrix_copy = Matrix;
  Matrix = 1/2 * (Matrix_copy - Matrix_copy.transpose());
  return Matrix;
}

Eigen::Matrix<double, 3, 6> regressor_helper_function(Eigen::Vector3d Vector){
  Eigen::Matrix<double, 3, 6> rhf = Eigen::MatrixXd::Zero(3, 6);
  rhf(0, 0) = Vector(0);
  rhf(1, 1) = Vector(0);
  rhf(2, 2) = Vector(0);
  rhf(0, 1) = Vector(1);
  rhf(1, 3) = Vector(1);
  rhf(2, 4) = Vector(1);
  rhf(0, 2) = Vector(2);
  rhf(1, 2) = Vector(2);
  rhf(2, 5) = Vector(2);

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
    initialized_params();

    // compute error signals
    Eigen::Matrix3d Re = R_d.transpose() * R;
    Eigen::Vector3d we;
    we = payload_angular_velocity - payload_reference_angular_velocity;

    // compute velocity error s
    Eigen::Matrix3d Pa_Re;
    Pa_Re = Pa(Re);
    Eigen::Vector3d Pa_Re_V;
    Pa_Re_V = vee_map(Pa_Re);
    Eigen::Vector3d sigma;
    sigma = we + lambda * R_d * Pa_Re_V;

    Eigen::Vector3d s_l;
    Eigen::Vector3d velocity_error = payload_linear_velocity - payload_reference_linear_velocity;
    Eigen::Vector3d position_error = payload_position - payload_reference_position;
    s_l = velocity_error + lambda * position_error;

    Eigen::Matrix<double, 6, 1> s;
    s.block<3, 1>(0, 0) = sigma;
    s.block<3, 1>(3, 0) = s_l;


    // compute Y_o
    // q_r dot is the combination of v_r and w_r
    // q_r double dot is the combination of a_r and alpha_r
    Eigen::Vector3d a_r = payload_reference_angular_acceleration - lambda * velocity_error;
    // not sure term
    Eigen::Vector3d al_r = payload_reference_angular_acceleration - lambda * hat_map(payload_reference_angular_velocity) * R_d * Pa_Re_V -
                                                                    lambda * R_d * vee_map(Pa(hat_map(R_d.transpose() * we)));
    Eigen::Vector3d w_r = payload_reference_angular_velocity - lambda * R_d * Pa_Re_V;
    Eigen::Vector3d v_r = payload_reference_linear_velocity - lambda * position_error;

    Eigen::Matrix<double, 3, 10> Y_l;
    Eigen::Matrix<double, 3, 10> Y_r;
    Eigen::Matrix<double, 6, 10> Y_o;

    Y_l.block<3, 1>(0, 0) = a_r;
    Y_l.block<3, 3>(0, 1) = -hat_map(al_r) * R - hat_map(payload_angular_velocity) * hat_map(w_r) * R;
    Y_l.block<3, 6>(0, 4) = Eigen::MatrixXd::Zero(3, 6);

    Y_r.block<3, 1>(0, 0) = Eigen::MatrixXd::Zero(3, 1);
    Y_r.block<3, 3>(0, 1) = hat_map(a_r) * R + hat_map(payload_angular_velocity) * hat_map(v_r) * R -
                                               hat_map(w_r) * hat_map(payload_linear_velocity) * R;
    Y_r.block<3, 6>(0, 4) = R * regressor_helper_function(R.transpose() * al_r) +
                            hat_map(payload_angular_velocity) * R * regressor_helper_function(R.transpose() * w_r);

    Y_o.block<3, 10>(0, 0) = Y_l;
    Y_o.block<3, 10>(3, 0) = Y_r;


    geometry_msgs::Wrench robot_cmd;
    robot_cmd.force.z = 0.5 / 2 * 9.81;
    robot_cmd.force.x = 0.1;
    // robot_cmd.torque.z = 3;

  	robot_controller_pub.publish(robot_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}