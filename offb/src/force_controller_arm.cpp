#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Inertia.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <numeric>
#include <algorithm>

# define USE_FORCE_SENSOR 0
# define USE_COMMAND_FORCE 1
# define FORCE_INPUT_SWITCHER USE_COMMAND_FORCE

// pose
Eigen::Matrix3d R;
Eigen::Matrix3d R_d;
Eigen::Vector3d r_i;

// state measurement
Eigen::Vector3d payload_linear_velocity;
Eigen::Vector3d payload_angular_velocity_b;
Eigen::Vector3d payload_angular_velocity;
Eigen::Vector3d payload_position;
double payload_roll, payload_yaw, payload_pitch;
Eigen::Matrix<double, 6, 1> true_tau;
Eigen::Vector3d true_f_b;
Eigen::Vector3d true_m_b;

// state reference
Eigen::Vector3d payload_reference_linear_velocity;
Eigen::Vector3d payload_reference_angular_velocity;
Eigen::Vector3d payload_reference_position;
Eigen::Vector3d payload_reference_linear_acceleration;
Eigen::Vector3d payload_reference_angular_acceleration;
Eigen::Vector3d desired_angle;

Eigen::Matrix<double, 13, 1> o_i_hat;
std::queue<Eigen::Matrix<double, 13, 1>> ICL_queue;
bool pop_on = false;
double g = 9.81;
Eigen::Vector3d G;
double control_rate = 20;
Eigen::Vector3d angle_error;
bool ICL_update = true;

// option
bool rotation = false;
bool ICL_update_switcher = true;

// gain
double lambda;
Eigen::Matrix<double, 13, 13> gamma_o;
Eigen::Matrix<double, 6, 6> K_d;
Eigen::Vector3d K_p; // heading angle
Eigen::Matrix<double, 13, 13> k_cl;
double N_o;
double adaptive_gain;

void initialized_params(){
  // initialize the desired rotation matrix
  R_d << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  lambda = 1.1;
  G << 0, 0, -g;

  double gamma_gain = 0.1, gamma_mass = 0.4, gamma_arm = 0.1; //0.6
  gamma_o = Eigen::Matrix<double, 13, 13>::Identity() * gamma_gain;
  gamma_o(0, 0) = gamma_mass;
  gamma_o.bottomRightCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * gamma_arm;

  double Kdl_gain = 9;//12;
  double Kdr_gain = 7;//2.5;
  K_d = Eigen::Matrix<double, 6, 6>::Identity();
  K_d.topLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * Kdl_gain;
  K_d.bottomRightCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * Kdr_gain;
  // K_d.bottomRightCorner(1, 1) = Eigen::Matrix<double, 1, 1>::Identity() * Kdr_gain;

  o_i_hat = Eigen::MatrixXd::Zero(13, 1);
  o_i_hat(0) = 2.5; // mass
  o_i_hat(1) = o_i_hat(0) * (-0.666666);
  o_i_hat(2) = o_i_hat(0) * (-0.666666);
  o_i_hat(4) = 0.8416666 / 3.0; // Ixx
  o_i_hat(7) = 4.8416666 / 3.0; // Iyy
  o_i_hat(9) = 6.8333333 / 3.0; // Izz
  o_i_hat(10) = r_i(0); //r_ix
  o_i_hat(11) = r_i(1); //r_iy
  o_i_hat(12) = r_i(2); //r_iz
  // o_i_hat(10) = 0; //r_ix
  // o_i_hat(11) = 0; //r_iy
  // o_i_hat(12) = 0; //r_iz

  K_p << 0, 0, 1.0;//2.5;
  N_o = 20;

  double k_cl_gain, k_cl_arm_gain, k_cl_mass;
  k_cl_mass = 0.012;
  k_cl_gain = 0.01;
  k_cl_arm_gain = 0.05;
  k_cl = Eigen::Matrix<double, 13, 13>::Identity() * k_cl_gain;
  k_cl(0, 0) = k_cl_mass;
  k_cl.bottomRightCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * k_cl_arm_gain;
  // k_cl(9, 9) = k_cl_gain;
  adaptive_gain = 1.0 / 1.0;
}

void payload_orientation_cb(const sensor_msgs::Imu::ConstPtr &msg){
  sensor_msgs::Imu payload_imu;
  payload_imu = *msg;
  Eigen::Quaterniond q;

  if(isnan(payload_imu.angular_velocity.x) == 0){
    q.x() = payload_imu.orientation.x;
    q.y() = payload_imu.orientation.y;
    q.z() = payload_imu.orientation.z;
    q.w() = payload_imu.orientation.w;

    R = q.normalized().toRotationMatrix();
    float lpf_gain = 0.2;

    payload_angular_velocity_b(0) = payload_imu.angular_velocity.x * lpf_gain + payload_angular_velocity_b(0) * (1-lpf_gain);
    payload_angular_velocity_b(1) = payload_imu.angular_velocity.y * lpf_gain + payload_angular_velocity_b(1) * (1-lpf_gain);
    payload_angular_velocity_b(2) = payload_imu.angular_velocity.z * lpf_gain + payload_angular_velocity_b(2) * (1-lpf_gain);

    payload_angular_velocity = R * payload_angular_velocity_b;

    tf::Quaternion quaternion(q.x(), q.y(), q.z(), q.w());
    tf::Matrix3x3(quaternion).getRPY(payload_roll, payload_pitch, payload_yaw);
  }else if(isnan(payload_imu.angular_velocity.x) != 0){
    std::cout << "I meet something cool like Imu nan!!" << std::endl;
  }
}

void reference_cb(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg){
  trajectory_msgs::MultiDOFJointTrajectoryPoint reference_input;
  reference_input = *msg;
  payload_reference_linear_velocity(0) = reference_input.velocities[0].linear.x;
  payload_reference_linear_velocity(1) = reference_input.velocities[0].linear.y;
  payload_reference_linear_velocity(2) = reference_input.velocities[0].linear.z;
  // payload_reference_angular_velocity(0) = 0;
  // payload_reference_angular_velocity(1) = 0;
  // payload_reference_angular_velocity(2) = 0;
  payload_reference_position(0) = reference_input.transforms[0].translation.x;
  payload_reference_position(1) = reference_input.transforms[0].translation.y;
  payload_reference_position(2) = reference_input.transforms[0].translation.z;

  float lpf_gain = 0.8;
  payload_reference_linear_acceleration(0) = reference_input.accelerations[0].linear.x * lpf_gain + payload_reference_linear_acceleration(0) * (1-lpf_gain);
  payload_reference_linear_acceleration(1) = reference_input.accelerations[0].linear.y * lpf_gain + payload_reference_linear_acceleration(1) * (1-lpf_gain);
  payload_reference_linear_acceleration(2) = reference_input.accelerations[0].linear.z * lpf_gain + payload_reference_linear_acceleration(2) * (1-lpf_gain);

  payload_reference_angular_acceleration(0) = 0;
  payload_reference_angular_acceleration(1) = 0;
  payload_reference_angular_acceleration(2) = 0;

  if(rotation == true){
    if(payload_reference_linear_velocity(0) != 0){
      desired_angle(0) = 0;
      desired_angle(1) = 0;
      desired_angle(2) = atan2(payload_reference_linear_velocity(1), payload_reference_linear_velocity(0));
    }
    angle_error(0) = desired_angle(0) - payload_pitch;
    angle_error(1) = desired_angle(1) - payload_roll;
    angle_error(2) = desired_angle(2) - payload_yaw;
    Eigen::Vector3d angle_error_i = R * angle_error;
    for(int i=0;i<3;i++){
      if(angle_error_i(i) > M_PI){
        angle_error_i(i) = angle_error_i(i) - 2 * M_PI;
      }else if(angle_error_i(i) < -M_PI){
        angle_error_i(i) = angle_error_i(i) + 2 * M_PI;
      }
      payload_reference_angular_velocity(i) = K_p(i) * angle_error_i(i);
    }
    // double bound = 10.0;
    // payload_reference_angular_acceleration(2) = K_p(i) * angle_error(2) * 0.05;
    // if(payload_reference_angular_acceleration(2) > bound){
    //   payload_reference_angular_acceleration(2) = bound;
    // }else if(payload_reference_angular_acceleration(2) < -bound){
    //   payload_reference_angular_acceleration(2) = -bound;
    // }

  }else{
    payload_reference_angular_velocity(0) = 0;
    payload_reference_angular_velocity(1) = 0;
    payload_reference_angular_velocity(2) = 0;
    payload_reference_angular_acceleration(2) = 0;
  }
}

void payload_odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
  nav_msgs::Odometry payload_odom;
  payload_odom = *msg;

  if(isnan(payload_odom.twist.twist.linear.x) != 0){
    std::cout << "I meet something cool like odom nan!!" << std::endl;
  }else{
    float lpf_gain = 1.0;
    payload_linear_velocity(0) = payload_odom.twist.twist.linear.x * lpf_gain + payload_linear_velocity(0) * (1-lpf_gain);
    payload_linear_velocity(1) = payload_odom.twist.twist.linear.y * lpf_gain + payload_linear_velocity(0) * (1-lpf_gain);
    payload_linear_velocity(2) = payload_odom.twist.twist.linear.z * lpf_gain + payload_linear_velocity(0) * (1-lpf_gain);
  }
  payload_position(0) = payload_odom.pose.pose.position.x;
  payload_position(1) = payload_odom.pose.pose.position.y;
  payload_position(2) = payload_odom.pose.pose.position.z;
}

void payload_ft_sensor_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  geometry_msgs::WrenchStamped payload_ft;
  payload_ft = *msg;
  float lpf_gain = 0.4;
  static double fz_last;
  true_f_b << payload_ft.wrench.force.x * lpf_gain + true_f_b(0) * (1.0-lpf_gain),
              payload_ft.wrench.force.y * lpf_gain + true_f_b(1) * (1.0-lpf_gain),
              payload_ft.wrench.force.z * 0.5 + fz_last * 0.5;

  true_m_b << payload_ft.wrench.torque.x * lpf_gain + true_m_b(0) * (1.0-lpf_gain),
              payload_ft.wrench.torque.y * lpf_gain + true_m_b(1) * (1.0-lpf_gain),
              payload_ft.wrench.torque.z * lpf_gain + true_m_b(2) * (1.0-lpf_gain);
  true_tau.block<3, 1>(0, 0) = R * true_f_b;
  true_tau.block<3, 1>(3, 0) = R * true_m_b;
  fz_last = payload_ft.wrench.force.z;
}

geometry_msgs::Inertia inertia_sum;
geometry_msgs::Inertia total_inertia_1;
geometry_msgs::Inertia total_inertia_2;
geometry_msgs::Inertia total_inertia_3;
void total_inertia_1_cb(const geometry_msgs::Inertia::ConstPtr &msg){
  total_inertia_1 = *msg;
}
void total_inertia_2_cb(const geometry_msgs::Inertia::ConstPtr &msg){
  total_inertia_2 = *msg;
}
void total_inertia_3_cb(const geometry_msgs::Inertia::ConstPtr &msg){
  total_inertia_3 = *msg;
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
  Eigen::Matrix3d Matrix_rt;
  Matrix_rt = 1/2 * (Matrix - Matrix.transpose());
  return Matrix_rt;
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
  rhf(1, 4) = Vector(2);
  rhf(2, 5) = Vector(2);
  return rhf;
}

Eigen::Matrix<double, 13, 1> ICL_queue_sum(std::queue<Eigen::Matrix<double, 13, 1>> queue){
  Eigen::Matrix<double, 13, 1> sum = Eigen::MatrixXd::Zero(13, 1);
  int size = queue.size();
  for(int i=0;i<size;i++){
    sum += queue.front();
    queue.pop();
  }
  return sum;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_controller");
  ros::NodeHandle nh;

  // get r_i for initial guess
  if(nh.getParam("r_i_x", r_i(0)) && nh.getParam("r_i_y", r_i(1)) && nh.getParam("r_i_z", r_i(2))){
    ROS_INFO("Received r_i info.");
  }else{
    ROS_WARN("Can't get r_i!");
  }


  ros::Subscriber payload_imu_sub = nh.subscribe<sensor_msgs::Imu>("/payload/IMU",4,payload_orientation_cb);
  ros::Subscriber reference_input_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/payload/desired_trajectory",4,reference_cb);
  ros::Subscriber payload_odom_sub = nh.subscribe<nav_msgs::Odometry>("/payload/position",4,payload_odom_cb);
  ros::Subscriber payload_ft_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/payload_ft_sensor",4,payload_ft_sensor_cb);

  // compute the total inertia
  ros::Subscriber total_inertia_1_sub = nh.subscribe<geometry_msgs::Inertia>("/robot_1/estimated",4,total_inertia_1_cb);
  ros::Subscriber total_inertia_2_sub = nh.subscribe<geometry_msgs::Inertia>("/robot_2/estimated",4,total_inertia_2_cb);
  ros::Subscriber total_inertia_3_sub = nh.subscribe<geometry_msgs::Inertia>("/robot_3/estimated",4,total_inertia_3_cb);

  ros::Publisher robot_controller_pub = nh.advertise<geometry_msgs::Wrench>("robot_wrench",4);
  // ros::Publisher estimated_m_pub = nh.advertise<geometry_msgs::Point>("/estimated/mass",4);
  // ros::Publisher estimated_I_pub = nh.advertise<geometry_msgs::Inertia>("/estimated/inertia",4);
  ros::Publisher estimated_pub = nh.advertise<geometry_msgs::Inertia>("/estimated",4);
  ros::Publisher pos_err_pub = nh.advertise<geometry_msgs::Pose2D>("/position_error",4);
  ros::Publisher arm_err_pub = nh.advertise<geometry_msgs::Point>("/arm_est_error",4);
  ros::Publisher s_norm_pub = nh.advertise<geometry_msgs::Point>("/s_norm",4);
  ros::Publisher estimated_sum_pub = nh.advertise<geometry_msgs::Inertia>("/estimated_sum",4);

  ros::Rate loop_rate(control_rate);
  initialized_params();
  double dt = 0;

  while(ros::ok()){

    if(rotation == true){
      R_d <<  cos(-desired_angle(2)), sin(-desired_angle(2)), 0,
             -sin(-desired_angle(2)), cos(-desired_angle(2)), 0,
                                   0,                      0, 1;
    }


    // compute error signals
    Eigen::Matrix3d Re = R_d.transpose() * R;
    Eigen::Vector3d we = payload_angular_velocity - payload_reference_angular_velocity;

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
    s.block<3, 1>(0, 0) = s_l;
    s.block<3, 1>(3, 0) = sigma;


    // compute Y_o
    // q_r dot is the combination of v_r and w_r
    // q_r double dot is the combination of a_r and alpha_r
    Eigen::Vector3d a_r = payload_reference_linear_acceleration - lambda * velocity_error;
    Eigen::Vector3d v_r = payload_reference_linear_velocity - lambda * position_error;
    // not sure term
    Eigen::Vector3d al_r = payload_reference_angular_acceleration - lambda * hat_map(payload_reference_angular_velocity) * R_d * Pa_Re_V -
                                                                    lambda * R_d * vee_map(Pa(hat_map(R_d.transpose() * we) * Re));
    Eigen::Vector3d w_r = payload_reference_angular_velocity - lambda * R_d * Pa_Re_V;
    Eigen::Vector3d pre_f_i;
    static Eigen::Matrix<double, 6, 1> wrench = Eigen::MatrixXd::Zero(6, 1);
#if (FORCE_INPUT_SWITCHER == USE_COMMAND_FORCE)
    pre_f_i << wrench(0), wrench(1), 0;
#elif (FORCE_INPUT_SWITCHER == USE_FORCE_SENSOR)
    pre_f_i << true_tau(0), true_tau(1), true_tau(2);// broken
#endif

    Eigen::Matrix<double, 3, 13> Y_l;
    Eigen::Matrix<double, 3, 13> Y_r;
    Eigen::Matrix<double, 6, 13> Y_o;

    Y_l.block<3, 1>(0, 0) = a_r;
    Y_l.block<3, 3>(0, 1) = -hat_map(al_r) * R - hat_map(payload_angular_velocity) * hat_map(w_r) * R;
    Y_l.block<3, 9>(0, 4) = Eigen::MatrixXd::Zero(3, 9);

    Y_r.block<3, 1>(0, 0) = Eigen::MatrixXd::Zero(3, 1);
    Y_r.block<3, 3>(0, 1) = hat_map(a_r) * R + hat_map(payload_angular_velocity) * hat_map(v_r) * R -
                                               hat_map(w_r) * hat_map(payload_linear_velocity) * R;
    Y_r.block<3, 6>(0, 4) = R * regressor_helper_function(R.transpose() * al_r) +
                            hat_map(payload_angular_velocity) * R * regressor_helper_function(R.transpose() * w_r);
    Y_r.block<3, 3>(0, 10) = hat_map(pre_f_i) * R;

    Y_o.block<3, 13>(0, 0) = Y_l;
    Y_o.block<3, 13>(3, 0) = Y_r;






    /*********/
    /*  ICL  */
    /*********/
    Eigen::Matrix<double, 3, 13> y_l;
    Eigen::Matrix<double, 3, 13> y_r;
    Eigen::Matrix<double, 6, 13> y_o_cl_integral;
    Eigen::Matrix<double, 13, 1> ICL_sum;

    static Eigen::Vector3d payload_linear_velocity_last;
    static Eigen::Vector3d payload_angular_velocity_last;
    static int index = 1;

    y_l.block<3, 1>(0, 0) = payload_linear_velocity - payload_linear_velocity_last;
    y_l.block<3, 3>(0, 1) = -hat_map(payload_angular_velocity - payload_angular_velocity_last) * R -
                             hat_map(payload_angular_velocity) * hat_map(payload_angular_velocity) * R * dt;
    y_l.block<3, 9>(0, 4) = Eigen::MatrixXd::Zero(3, 9);

    y_r.block<3, 1>(0, 0) = Eigen::MatrixXd::Zero(3, 1);
    y_r.block<3, 3>(0, 1) = hat_map(payload_linear_velocity - payload_linear_velocity_last) * R;
    y_r.block<3, 6>(0, 4) = R * regressor_helper_function(R.transpose() * (payload_angular_velocity - payload_angular_velocity_last)) +
                            hat_map(payload_angular_velocity) * R * regressor_helper_function(R.transpose() * payload_angular_velocity) * dt;
    y_r.block<3, 3>(0, 10) = hat_map(pre_f_i) * R * dt;

    y_o_cl_integral.block<3, 13>(0, 0) = y_l;
    y_o_cl_integral.block<3, 13>(3, 0) = y_r;

    // compute true tau and integral
#if (FORCE_INPUT_SWITCHER == USE_COMMAND_FORCE)
    Eigen::Matrix<double, 6, 1> true_tau_integral = wrench * dt;
#elif (FORCE_INPUT_SWITCHER == USE_FORCE_SENSOR)
#pragma message("Force Sensor!")
    Eigen::Matrix<double, 6, 1> true_tau_integral = true_tau * dt;
#endif

    Eigen::Matrix<double, 6, 1> y_o_cl_integral_o_i_hat = y_o_cl_integral * o_i_hat;

    // determine whether arrive the N_o
    if(index > N_o){
      pop_on = true;
      ICL_queue.pop();
    }
    // if(pop_on == true){
    //   ICL_queue.pop();
    // }
    if(dt!=0){  // if not in the first loop
      ICL_queue.push(y_o_cl_integral.transpose() * (y_o_cl_integral_o_i_hat - true_tau_integral));
    }
    ICL_sum = ICL_queue_sum(ICL_queue);

    payload_linear_velocity_last = payload_linear_velocity;
    payload_angular_velocity_last = payload_angular_velocity;
    index++;
    /*********/
    /*  ICL  */
    /*********/




    // compute o_i hat
    Eigen::Matrix<double, 13, 1> o_i_hat_dot = Eigen::MatrixXd::Zero(13, 1);
    if(ICL_update == true && ICL_update_switcher == true){
      o_i_hat_dot = -adaptive_gain * gamma_o * Y_o.transpose() * s - k_cl * gamma_o * ICL_sum;
    }else{
      o_i_hat_dot = -adaptive_gain * gamma_o * Y_o.transpose() * s;
    }

    // implement the adaptation law
    o_i_hat = o_i_hat + o_i_hat_dot * dt;

    // implement control law
    Eigen::Matrix<double, 6, 1> F_i = Y_o * o_i_hat - K_d * s;

    // transfer to wrench
    wrench = F_i;



    geometry_msgs::Wrench robot_cmd;
    if(isnan(wrench(0)) != 0){
      std::cout << "I meet something cool like wrench nan!!" << std::endl;
      std::cout << "Please restart the controller and try again." << std::endl;
    }else{
      robot_cmd.force.x = wrench(0);
      robot_cmd.force.y = wrench(1);
      robot_cmd.force.z = 0;
      // robot_cmd.force.z = 0.8 * 8.0 / 3.0 * g;
      // robot_cmd.force.z = 0.65 * true_tau(2);
      robot_cmd.torque.x = wrench(3);
      robot_cmd.torque.y = wrench(4);
      robot_cmd.torque.z = wrench(5);
    }

    // debug output
    std::cout << "-------" << std::endl;
    std::cout << "m sum: " << inertia_sum.m << std::endl << std::endl;
    std::cout << "m: " << o_i_hat(0) << std::endl << std::endl;
    // std::cout << "Ixx: " << o_i_hat(4) << std::endl << std::endl;
    // std::cout << "Iyy: " << o_i_hat(7) << std::endl << std::endl;
    // std::cout << "Izz: " << o_i_hat(9) << std::endl << std::endl;
    // std::cout << "-k_cl * gamma_o * ICL_sum: " << -k_cl * gamma_o * ICL_sum << std::endl << std::endl;
    std::cout << "wrench: " << wrench << std::endl << std::endl;
    // std::cout << "ICL: " << ICL_sum << std::endl << std::endl;
    // std::cout << "r_px: " << o_i_hat(1) / o_i_hat(0) << std::endl << std::endl;
    // std::cout << "r_py: " << o_i_hat(2) / o_i_hat(0) << std::endl << std::endl;
    // std::cout << "r_pz: " << o_i_hat(3) / o_i_hat(0) << std::endl << std::endl;
    // std::cout << "Y_o * o_i_hat: " << Y_o * o_i_hat << std::endl << std::endl;
    std::cout << "o_i_hat: " << o_i_hat << std::endl << std::endl;
    // std::cout << "Y_o: " << Y_o << std::endl << std::endl;
    // std::cout << "hat_map(pre_f_i) * R: " << hat_map(pre_f_i) * R << std::endl << std::endl;
    std::cout << "true_f_b(2): " << true_f_b(2) << std::endl << std::endl;

    std::cout << "r_i: " << o_i_hat(10) << "  " <<o_i_hat(11) << "  "  << o_i_hat(12) <<std::endl;
    std::cout << "ri ground truth: " << std::endl << r_i << std::endl << std::endl;
    std::cout << "-------" << std::endl;

    // plot output
    // pose error
    geometry_msgs::Pose2D pos_error;
    // pos_error.x = position_error(0);
    // pos_error.y = position_error(1);
    // pos_error.theta = angle_error;
    pos_error.x = true_tau(0);
    pos_error.y = true_tau(1);
    pos_error.theta = true_tau(2);

    // velocity error s
    geometry_msgs::Point s_norm;
    double s_n = 0;
    for(int i=0;i<s.size();i++){
      s_n += s(i) * s(i);
    }
    s_norm.x = s_n;
    if(s_n < 0.02){
      ICL_update = false;
    }else{
      ICL_update = true;
    }

    s_n = 0;
    for(int i=0;i<3;i++){
      s_n += s(i) * s(i);
    }
    s_norm.y = s_n; //l
    s_n = 0;
    for(int i=3;i<6;i++){
      s_n += s(i) * s(i);
    }
    s_norm.z = s_n; //r

    // arm estimated error
    geometry_msgs::Point arm_error;
    arm_error.x = o_i_hat(10) - r_i(0);
    arm_error.y = o_i_hat(11) - r_i(1);
    arm_error.z = sqrt(arm_error.x * arm_error.x) + (arm_error.y * arm_error.y);

    dt = 1 / control_rate;

    // Inertia
    geometry_msgs::Inertia inertia;
    inertia.m = o_i_hat(0);
    inertia.ixx = o_i_hat(4);
    inertia.ixy = o_i_hat(5);
    inertia.ixz = o_i_hat(6);
    inertia.iyy = o_i_hat(7);
    inertia.iyz = o_i_hat(8);
    inertia.izz = o_i_hat(9);

    // Inertia sum
    inertia_sum.m = total_inertia_1.m + total_inertia_2.m + total_inertia_3.m;
    inertia_sum.ixx = total_inertia_1.ixx + total_inertia_2.ixx + total_inertia_3.ixx;
    inertia_sum.ixy = total_inertia_1.ixy + total_inertia_2.ixy + total_inertia_3.ixy;
    inertia_sum.ixz = total_inertia_1.ixz + total_inertia_2.ixz + total_inertia_3.ixz;
    inertia_sum.iyy = total_inertia_1.iyy + total_inertia_2.iyy + total_inertia_3.iyy;
    inertia_sum.iyz = total_inertia_1.iyz + total_inertia_2.iyz + total_inertia_3.iyz;
    inertia_sum.izz = total_inertia_1.izz + total_inertia_2.izz + total_inertia_3.izz;

  	robot_controller_pub.publish(robot_cmd);
    // estimated_m_pub.publish(mass);
    estimated_pub.publish(inertia);
    pos_err_pub.publish(pos_error);
    arm_err_pub.publish(arm_error);
    s_norm_pub.publish(s_norm);
    estimated_sum_pub.publish(inertia_sum);

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}