#pragma once
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mav_planning/environment.hpp>
#include <mav_planning/tracker.hpp>
#include <mav_planning/explorer.hpp>
#include <mav_planning/replanner.hpp>
#include <mav_planning/inspector.hpp>
#include <mav_planning/lander.hpp>
#include <mav_planning/planner.hpp>
#include <mav_planning/containers.hpp>

#include <mav_planning/Action.h>
#include <mav_regulator/Point.h>
#include <mav_regulator/SetPoint.h>
#include <mav_regulator/BSpline.h>
#include <mav_regulator/BezierMultiArray.h>
#include <geometry_msgs/PoseStamped.h>

typedef enum{SPLINE, POINT} SetPointType;

typedef enum{PLAN_ACTION,
			 DO_INIT_ACTION,
			 WAIT_INIT_ACTION,
			 DO_MAIN_ACTION,
			 SUPERVISE_ACTION,
			 DO_SECOND_INIT,
			 WAIT_SECOND_INIT,
			 DO_SECOND_ACTION} ActionState;

class MavPlanner{
	private:
	bool action_valid = false;
	bool first_entry = true;
	
	bool isFirstTime_lander = true;
	bool isFirstTime_timer = true;

	bool use_replanner;
	bool debug_flag;

	double takeoff_altitude;
	double takeoff_velocity;
	double rotation_velocity;
	double safe_margin;
	double yy_thr, yy_sp, yy_map;

	double zz_eps, tt_delay;
	double zz_init;

	int nn_cicle_wait;
	int waitCount = 0;

	Environment* environment;
	_Tracker_::Tracker* tracker;
	_Explorer_::Explorer* explorer;
	_Replanner_::Replanner* replanner;
	_Inspector_::Inspector* inspector;
	_Planner_::Planner* planner;
	_Lander_::Lander* lander;

	ros::Subscriber actSubscriber;
	ros::Publisher setPointPublisher;
	ros::Publisher splinePublisher;
	ros::Publisher bezierPublisher;
	ros::Publisher statePublisher;
	ros::Timer actTimer;

    tf2::Transform odomToMap_transformation;
	tf2::Transform bodyToOdom_transformation;

	ros::Time initialTime = ros::Time::now();
	ros::Time tt_init = ros::Time::now();

	mav_planning::Action currentAction;
	ActionState currentState;
	SetPointType currentSetPoint;

	Eigen::Matrix<double, 3, 1> actualPosition;
	Eigen::Matrix<double, 3, 1> actualSetPoint;
	double actualOrientation;

	BSpline<Eigen::Matrix<double, 3, 1>>* pp_spline;
	BSpline<double>* yy_spline;

	std::vector<Eigen::Matrix<double, 3, 1>> cp_mm;
	std::vector<double> cpy_mm;
	double tt_mm;

	void exec(const ros::TimerEvent& e);
	bool readActualOdometry();
	void triggerStop();
	bool isActionExecuted();
	bool isLanded();
	void superviseAction(double dt);
	void sendSpline(int act);
	void sendSpline_withYaw(int act);
	void sendPoint(double x, double y, double z, double yy, double tt, bool reset);
	void checkPointCollision(Eigen::Matrix<double, 3, 1>& pp);
	void actionCallback(const mav_planning::Action::ConstPtr& msg);

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
	MavPlanner(ros::NodeHandle& nh);

	// Class Destructor
	~MavPlanner();
};
