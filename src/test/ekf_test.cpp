#include <mav_planning/tracker.hpp>
#include <visualization_msgs/MarkerArray.h>

using namespace _Tracker_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_ekf");
	ros::NodeHandle nh("~");
  	ros::Rate rate(5.0);

	ros::Publisher visualPublisher_r = nh.advertise<visualization_msgs::MarkerArray>("/traj", 1);
	ros::Publisher visualPublisher_k = nh.advertise<visualization_msgs::MarkerArray>("/kalman", 1);

	std::srand(std::time(nullptr));

	EKF kalman(nh);

	Eigen::Matrix<double, 5, 1> xk;
	xk << 0, 0, 1, 0, 0;

	Eigen::Matrix<double, 5, 1> xn;
	xn << 0, 0, 0, 0, 0;

	kalman.reset(xn);

	double max_acc_1 = 0.5;
	double max_acc_2 = 0.1;

	ros::Time t1 = ros::Time::now();
	ros::Time t2 = ros::Time::now();

	int id = 0;
	while(ros::ok()){
		// Generate New State
		double u1 = (((double)std::rand()/(double)RAND_MAX)*2*max_acc_1) - max_acc_1;
		double u2 = (((double)std::rand()/(double)RAND_MAX)*2*max_acc_2) - max_acc_2;

		double n1 = (((double)std::rand()/(double)RAND_MAX)*2*0.1) - 0.1;
		double n2 = (((double)std::rand()/(double)RAND_MAX)*2*0.1) - 0.1;
		double n3 = (((double)std::rand()/(double)RAND_MAX)*2*0.05) - 0.05;

		Eigen::Matrix<double, 5, 1> xd;
		xd(0) = cos(xk(3))*xk(2); 
		xd(1) = sin(xk(3))*xk(2); 
		xd(2) = u1;
		xd(3) = xk(4);
		xd(4) = u2;

		// Euler Step
		t1 = ros::Time::now();
		xk = xk + (xd*(t1-t2).toSec());
		t2 = t1;

		Eigen::Matrix<double, 3, 1> yy;
		Eigen::Matrix<double, 3, 3> rr;
		Eigen::Matrix<double, 5, 1> xx;

		for(uint ii = 0; ii < 3; ii++){
			for(uint jj = 0; jj < 3; jj++){
				if(ii == jj){
					rr(ii,jj) = 1.0;
				}
			}
		}

		yy(0) = xk(0); // + n1;
		yy(1) = xk(1); // + n2;
		yy(2) = xk(3); // + n3;

		kalman.update(yy, rr);
		kalman.getState(xx);

		ROS_INFO("[EKF TEST]: Current Robot State %f %f %f %f %f", xk(0), xk(1), xk(2), xk(3), xk(4));
		ROS_INFO("[EKF TEST]: Current Kalman State %f %f %f %f %f", xx(0), xx(1), xx(2), xx(3), xx(4));

		visualization_msgs::MarkerArray trajPoint;
		visualization_msgs::MarkerArray kalmanPoint;
		visualization_msgs::Marker point;
		geometry_msgs::Point pp;
		pp.x = xk(0);
		pp.y = xk(1);
		pp.z = 0.0;

		point.header.frame_id = "map";
		point.header.stamp = ros::Time::now();
		point.ns = "ns";
		point.id = id++;
		point.type = visualization_msgs::Marker::SPHERE_LIST;
		point.points.push_back(pp);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 1.0;
		point.color.r = 0.0;
		point.color.g = 1.0;
		point.color.b = 0.0;
		point.action = visualization_msgs::Marker::ADD;

		trajPoint.markers.push_back(point);
		visualPublisher_r.publish(trajPoint);

		pp.x = xx(0);
		pp.y = xx(1);
		point.header.frame_id = "map";
		point.header.stamp = ros::Time::now();
		point.ns = "ns";
		point.id = id++;
		point.type = visualization_msgs::Marker::SPHERE_LIST;
		point.points.push_back(pp);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 1.0;
		point.color.r = 1.0;
		point.color.g = 0.0;
		point.color.b = 0.0;
		point.action = visualization_msgs::Marker::ADD;


		kalmanPoint.markers.push_back(point);
		visualPublisher_k.publish(kalmanPoint);
		
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}