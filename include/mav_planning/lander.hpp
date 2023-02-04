#pragma once
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <vector>

#include <mav_planning/environment.hpp>
#include <mav_planning/containers.hpp>
#include <mav_planning/optimizer.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

namespace _Lander_{

typedef struct{
	double dd;
	uint idx;
} DistanceIdx;

class Lander{
	private:
	bool haveCamInfo = false;
	bool processing_image = false;
	bool image_valid = false;

	bool debug_flag;
	int cx, cy;
	double land_vv, end_zz;
	double th_thr, rh_thr, ortho_thr;

	uint idx1, idx2, idx3, idx4;
	double scx, scy;

	double ll_px, ll_mm, mm_px, thr_perc;

	double new_xx_position, new_yy_position;
	double ee_xx_mm, ee_yy_mm;

	Environment* environment;
	Optimizer* optimizer;
	BSpline<Eigen::Matrix<double, 3, 1>>* spline;

	cv::Mat img;
	cv::Mat cm = cv::Mat(3, 3, cv::DataType<double>::type);
    cv::Mat dc = cv::Mat(1, 4, cv::DataType<double>::type);

	Eigen::Matrix<double, 1, 3> actualPosition;
	double actualYaw;

	ros::Subscriber imageSubscriber;
	ros::Subscriber infoSubscriber;
	ros::Publisher imagePublisher;

	tf2::Transform odomToMap_transformation;
	tf2::Transform bodyToOdom_transformation;

	void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
	void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

	bool processImage(std::vector<cv::Point>& pps);
	void simpleLanding();
	void computeErrorSquare(std::vector<cv::Point> pps);
	bool checkSquare();

	cv::Point findIntersection(cv::Vec2f l1, cv::Vec2f l2);
	bool findSquare(std::vector<cv::Point> pps, uint& idx1, uint& idx2, uint& idx3, uint& idx4);
	void combineElements(std::vector<DistanceIdx> in, std::vector<std::vector<DistanceIdx>>& out);

	static bool sortDistances(DistanceIdx di1, DistanceIdx di2){ return di1.dd < di2.dd; };

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
			yy = yy + M_PI;
		}
		
		while(yy > M_PI){
			yy = yy - M_PI;
		}

		return yy;
	};

	public:
	// Class Constructor
	Lander(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Lander();

	bool land(std::vector<Eigen::Matrix<double, 3, 1>>& cp, std::vector<double>& cpy, double& tt, double dt);
	void reset();
};
}