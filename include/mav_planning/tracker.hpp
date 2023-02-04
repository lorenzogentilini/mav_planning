#pragma once
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <vector>

#include <mav_planning/environment.hpp>
#include <mav_planning/containers.hpp>
#include <mav_planning/optimizer.hpp>

#include <nav_msgs/Odometry.h>

namespace _Tracker_{
class EKF{
	private:
	bool ready = false;

	double dv, dw;

	Eigen::Matrix<double, 5, 1> xk;
	Eigen::Matrix<double, 5, 5> pk;
	Eigen::Matrix<double, 5, 5> pi;
	Eigen::Matrix<double, 5, 5> qq;
	Eigen::Matrix<double, 3, 5> hh;
	Eigen::Matrix<double, 5, 5> id;

	Eigen::Matrix<double, 5, 1> pd;
	Eigen::Matrix<double, 5, 1> qd;

	ros::Time previousIter;
	ros::Time actualIter;

	void getSystemDynamics(Eigen::Matrix<double, 5, 1>& xd, Eigen::Matrix<double, 5, 1> xx);
	void getJacobian(Eigen::Matrix<double, 5, 5>& ff, Eigen::Matrix<double, 5, 1> xx);

	public:
	// Class Constructor
	EKF(ros::NodeHandle& nh);

	// Class Destructor
	~EKF();

	void update();
	void correct(Eigen::Matrix<double, 3, 1> yk, Eigen::Matrix<double, 3, 3> rk);
	void predict(std::vector<Eigen::Matrix<double, 5, 1>>& xx, double dt, double tt);
	void getState(Eigen::Matrix<double, 5, 1>& xx){ xx = xk; };
	void reset(Eigen::Matrix<double, 5, 1> xx);
};

class Tracker{
	private:
	int robot_id, n_pp;
	double max_dt, rb_rr, mm_res, tt_max;
	double takeoff_altitude, safe_margin;
	double tt;

	double KP, KI;
	double KP_yy, KI_yy;

	bool debug_flag;

	bool poseValid = false;
	bool newPose = false;

	ros::Time last_tt;

	std::vector<Eigen::Matrix<double, 2, 1>> rp;
	std::vector<double> rp_yy;

	std::vector<Eigen::Matrix<double, 3, 1>> pps;
	std::vector<double> pps_yy;

	std::vector<Eigen::Matrix<double, 3, 1>> wp;
	std::vector<double> wp_yy;

	std::vector<double> wp_tt;
	Eigen::Matrix<double, 3, 1> track_error = Eigen::MatrixXd::Zero(3,1);
	double track_error_yy = 0.0;

	EKF* kalman;
	Environment* environment;
	Optimizer* optimizer;

	BSpline<Eigen::Matrix<double, 3, 1>>* spline;
	BSpline<double>* spline_yy;

	tf2::Transform mapToOdom_transformation;
	tf2::Transform mapToBody_transformation;

	ros::Subscriber odomSubscriber;
	ros::Subscriber poseSubscriber;

	void poseCallback(const nav_msgs::Odometry::ConstPtr& pose);
	void predictRoverPosition(Eigen::Matrix<double, 3, 1>& init_vv, Eigen::Matrix<double, 3, 1>& last_vv);
	void getWaypoints();

	double to_2PI(double yy){
		while(yy < 0){
			yy = yy + 2*M_PI;
		}
		
		while(yy >= 2*M_PI){
			yy = yy - 2*M_PI;
		}

		return yy;
	};

	double to_PI(double yy){
		while(yy < -M_PI){
			yy = yy + 2*M_PI;
		}
		
		while(yy > M_PI){
			yy = yy - 2*M_PI;
		}

		return yy;
	};
	
	public:
	// Class Constructor
	Tracker(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Tracker();
	
	bool track(double ti, std::vector<Eigen::Matrix<double, 3, 1>>& _cp, std::vector<double>& _cpy, double& _tt);
	void getPredictedRoverPose(std::vector<Eigen::Matrix<double, 3, 1>>& _rp, std::vector<double>& _ry);
	void updateEKF_forDebug(Eigen::Matrix<double, 3, 1> yy);
	void reset();
};
}