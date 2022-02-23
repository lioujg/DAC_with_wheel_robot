#include <ros/ros.h>
#include <geometric_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <qptrajectory.h>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#define normal
#define PI 3.14159


gazebo_msgs::ModelStates model_states;
float vir_x, vir_y,vir_z,vx,vy,vz,ax,ay,az;
unsigned int tick=0;
unsigned int counter = 0;
bool flag = false;
mavros_msgs::State current_state;
gazebo_msgs::LinkStates link_states;
geometry_msgs::PoseStamped desired_pose;
double tt;
double r = 2;
double T = 280*M_PI;


int main(int argc, char **argv)
{

	ros::init(argc, argv, "geo");
	ros::NodeHandle nh;

	ROS_INFO("Hello world!");

	ros::Publisher  traj_pub= nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/payload/desired_trajectory", 1);

	double dt = 20.0;
	ros::Rate loop_rate(dt);
	nh.setParam("/start",false);
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj;

	//planning
	qptrajectory plan;
	path_def path;
	trajectory_profile p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;
	std::vector<trajectory_profile> data;
	double sample = 0.05, swift = -0.5;


#if 1
	p1.pos << 0.5+swift,0,0.46;
    p1.vel << 0,0,0;
    p1.acc << 0,0,0;
    p1.yaw = 0;

    p2.pos << 3.5*2,5*2+swift,0.46;
    p2.vel << 0,0,0;
    p2.acc << 0,0,0;
    p2.yaw = 1;

    p3.pos << 13.5*2+swift,0,0.46;
    p3.vel << 0,0,0;
    p3.acc << 0,0,0;
    p3.yaw = 0;

    p4.pos << 3.5*2+swift,-5*2,0.46;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << -2.5*2,5*2+swift,0.46;
    p5.vel << 0,0,0;
    p5.acc << 0,0,0;
    p5.yaw = 0;

    p6.pos << -12.5*2,0+swift,0.46;
    p6.vel << 0,0,0;
    p6.acc << 0,0,0;
    p6.yaw = 0;

    p7.pos << -2.5*2+swift,-5*2,0.46;
    p7.vel << 0,0,0;
    p7.acc << 0,0,0;
    p7.yaw = 0;

    p8.pos << 0.5+swift,0,0.46;
    p8.vel << 0,0,0;
    p8.acc << 0,0,0;
    p8.yaw = 0;

  float duration = 16.0;
  path.push_back(segments(p1,p2,duration));
  path.push_back(segments(p2,p3,duration));
  path.push_back(segments(p3,p4,duration));
  path.push_back(segments(p4,p5,duration));
  path.push_back(segments(p5,p6,duration));
  path.push_back(segments(p6,p7,duration));
  path.push_back(segments(p7,p8,duration));

#elif 0
	p1.pos << 0.5,0,0.46;
    p1.vel << 0,0,0;
    p1.acc << 0,0,0;
    p1.yaw = 0;

    p2.pos << 3.5,5,0.46;
    p2.vel << 0,0,0;
    p2.acc << 0,0,0;
    p2.yaw = 1;

    p3.pos << 13.5,0,0.46;
    p3.vel << 0,0,0;
    p3.acc << 0,0,0;
    p3.yaw = 0;

    p4.pos << 3.5,-5,0.46;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << -2.5,5,0.46;
    p5.vel << 0,0,0;
    p5.acc << 0,0,0;
    p5.yaw = 0;

    p6.pos << -12.5,0,0.46;
    p6.vel << 0,0,0;
    p6.acc << 0,0,0;
    p6.yaw = 0;

    p7.pos << -2.5,-5,0.46;
    p7.vel << 0,0,0;
    p7.acc << 0,0,0;
    p7.yaw = 0;

    p8.pos << 0.5,0,0.46;
    p8.vel << 0,0,0;
    p8.acc << 0,0,0;
    p8.yaw = 0;

  float duration = 5.0;
  path.push_back(segments(p1,p2,duration));
  path.push_back(segments(p2,p3,duration));
  path.push_back(segments(p3,p4,duration));
  path.push_back(segments(p4,p5,duration));
  path.push_back(segments(p5,p6,duration));
  path.push_back(segments(p6,p7,duration));
  path.push_back(segments(p7,p8,duration));
  path.push_back(segments(p8,p2,4));
  path.push_back(segments(p2,p3,duration));
  path.push_back(segments(p3,p4,duration));
  path.push_back(segments(p4,p5,duration));
  path.push_back(segments(p5,p6,duration));
  path.push_back(segments(p6,p7,duration));
  path.push_back(segments(p7,p8,duration));

#elif 0
  	p1.pos << 0.5,0,0.46;
    p1.vel << 0,0,0;
    p1.acc << 0,0,0;
    p1.yaw = 0;

    p2.pos << 3.5,5,0.46;
    p2.vel << 0,0,0;
    p2.acc << 0,0,0;
    p2.yaw = 0;

    p3.pos << 7,-5,0.46;
    p3.vel << 0,0,0;
    p3.acc << 0,0,0;
    p3.yaw = 0;

    p4.pos << 12.5,0,0.46;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << 7,5,0.46;
    p5.vel << 0,0,0;
    p5.acc << 0,0,0;
    p5.yaw = 0;

    p6.pos << 3.5,-5,0.46;
    p6.vel << 0,0,0;
    p6.acc << 0,0,0;
    p6.yaw = 0;

    p7.pos << -2.5,5,0.46;
    p7.vel << 0,0,0;
    p7.acc << 0,0,0;
    p7.yaw = 0;

    p8.pos << -6,-5,0.46;
    p8.vel << 0,0,0;
    p8.acc << 0,0,0;
    p8.yaw = 0;

    p9.pos << -11.5,0,0.46;
    p9.vel << 0,0,0;
    p9.acc << 0,0,0;
    p9.yaw = 0;

    p10.pos << -6,5,0.46;
    p10.vel << 0,0,0;
    p10.acc << 0,0,0;
    p10.yaw = 0;

    p11.pos << -2.5,-5,0.46;
    p11.vel << 0,0,0;
    p11.acc << 0,0,0;
    p11.yaw = 0;

    p12.pos << 0.5,0,0.46;
    p12.vel << 0,0,0;
    p12.acc << 0,0,0;
    p12.yaw = 0;

  float duration = 6.0;
  path.push_back(segments(p1,p2,duration));
  path.push_back(segments(p2,p3,duration));
  path.push_back(segments(p3,p4,duration));
  path.push_back(segments(p4,p5,duration));
  path.push_back(segments(p5,p6,duration));
  path.push_back(segments(p6,p7,duration));
  path.push_back(segments(p7,p8,duration));
  path.push_back(segments(p8,p9,duration));
  path.push_back(segments(p9,p10,duration));
  path.push_back(segments(p10,p11,duration));
  path.push_back(segments(p11,p12,duration));
#endif


	data = plan.get_profile(path, path.size(), sample);

	desired_pose.pose.position.x = 0.5+swift;
	desired_pose.pose.position.y = 0.0;
	desired_pose.pose.position.z = 0.46;

	geometry_msgs::Transform transform;
	geometry_msgs::Twist twist;
	transform.translation.x = 0;
	transform.translation.y = 0;
	transform.translation.z = 0;
	transform.rotation.x = 0;
	transform.rotation.y = 0;
	transform.rotation.z = 0;
	transform.rotation.w = 0;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	traj.transforms.push_back(transform);
	traj.velocities.push_back(twist);
	traj.accelerations.push_back(twist);

	while(ros::ok()) {

		if(nh.getParam("/start",flag)) {

		};

		if(flag == false || (tick>data.size())) {

			//do position control
			nh.setParam("/start",false);
			tick = 0;
			vir_x = desired_pose.pose.position.x;
			vir_y = desired_pose.pose.position.y;
			vir_z = desired_pose.pose.position.z;

			vx = 0;
			vy = 0;
			vz = 0;

			ax = 0;
			ay = 0;
			az = 0;
			nh.setParam("/start",true);
		} else {

			vir_x = data[tick].pos(0);
			vir_y = data[tick].pos(1);
			vir_z = data[tick].pos(2);
			vx = data[tick].vel(0);
			vy = data[tick].vel(1);
			vz = data[tick].vel(2);
			ax = data[tick].acc(0);
			ay = data[tick].acc(1);
			az = data[tick].acc(2);
			tick++;

			double t = ros::Time::now().toSec();
			//std::cout << "time is " << t << std::endl;
#if 0
			vir_x = 0;
			vir_y = 0;
			vir_z = desired_pose.pose.position.z;// + 0.3*sin(1.4*M_PI*t/T);

			vx = 0;
			vy = 0;
			vz = 0;//0.3*cos(1.4*M_PI*t/T)*1.4*M_PI*t/T;

			ax = 0;
			ay = 0;
			az = 0;//-0.3*sin(1.4*M_PI*t/T)*1.4*M_PI/T*1.4*M_PI/T;
#endif
		}

#if 1
		traj.transforms[0].translation.x = vir_x;
		traj.transforms[0].translation.y = vir_y;
		traj.transforms[0].translation.z = vir_z;
		traj.velocities[0].linear.x = vx;
		traj.velocities[0].linear.y = vy;
		traj.velocities[0].linear.z = vz;
		traj.accelerations[0].linear.x = ax;
		traj.accelerations[0].linear.y = ay;
		traj.accelerations[0].linear.z = az;
#else
		float slope = tan(0.03);
		traj.transforms[0].translation.x = vir_x;
		traj.transforms[0].translation.y = vir_y;
		traj.transforms[0].translation.z = fabs(vir_x) * slope;
		traj.velocities[0].linear.x = vx;
		traj.velocities[0].linear.y = vy;
		traj.accelerations[0].linear.x = ax;
		traj.accelerations[0].linear.y = ay;
		if(vir_x > 0){
			traj.velocities[0].linear.z = vx * slope;
			traj.accelerations[0].linear.z = ax * slope;
		}else if(vir_x < 0){
			traj.velocities[0].linear.z = -vx * slope;
			traj.accelerations[0].linear.z = -ax * slope;
		}else{
			traj.velocities[0].linear.z = 0;
			traj.accelerations[0].linear.z = 0;
		}
#endif

		traj_pub.publish(traj);

		ros::spinOnce();
		loop_rate.sleep();
	}

}
