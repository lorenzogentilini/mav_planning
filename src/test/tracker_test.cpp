#include <mav_planning/tracker.hpp>
#include <visualization_msgs/MarkerArray.h>

using namespace _Tracker_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_tracker");
	ros::NodeHandle nh("~");

	ros::Publisher visualPublisher_pp = nh.advertise<visualization_msgs::MarkerArray>("/points", 1);
	ros::Publisher visualPublisher_ps = nh.advertise<visualization_msgs::Marker>("/pose", 1);
	ros::Publisher visualPublisher_tt = nh.advertise<visualization_msgs::MarkerArray>("/traj", 1);

	Environment* environment = new Environment(nh);
	Tracker tracker(nh, environment);

  	ros::Rate rate(2);
	std::srand(std::time(nullptr));

	ros::Time t1 = ros::Time::now();
	ros::Time t2 = ros::Time::now();

	Eigen::Matrix<double, 5, 1> xk;
	xk << -13.5, 5.5, 0.5, 0.0, 0.0;
	for(uint ii = 0; ii < 50; ii++){
		Eigen::Matrix<double, 5, 1> xd;

		double ma = 0.2;
		double mw = 0.1;

		xd(0) = cos(xk(3))*xk(2); 
		xd(1) = sin(xk(3))*xk(2); 
		xd(2) = ((double)std::rand()/(double)RAND_MAX)*2*ma - ma;
		xd(3) = xk(4);
		xd(4) = ((double)std::rand()/(double)RAND_MAX)*2*mw - mw;

		// Euler Step
		t1 = ros::Time::now();
		xk = xk + xd*(t1-t2).toSec();

		Eigen::Matrix<double, 3, 1> yy;
		yy(0) = xk(0);
		yy(1) = xk(1);
		yy(2) = xk(3);

		tracker.updateEKF_forDebug(yy);

		ROS_INFO("[TRACKER TEST]: Current Robot State %f %f %f %f %f", xk(0), xk(1), xk(2), xk(3), xk(4));
		ROS_INFO("[TRACKER TEST]: Start Tracking");
		ros::Time tr1 = ros::Time::now();
		std::vector<Eigen::Matrix<double, 3, 1>> cp;
		std::vector<double> cpy;
		double tt;
		if(!tracker.track((t1-t2).toSec(), cp, cpy, tt)){
			continue;
		}

		ros::Time tr2 = ros::Time::now();
		ROS_INFO("[TRACKER TEST]: End Tracking, Elapsed in %f", (tr2-tr1).toSec());

		for(uint ii = 0; ii < cp.size(); ii++){
			ROS_INFO("[TRACKER TEST]: Get %f %f %f", cp[ii](0), cp[ii](1), cp[ii](2));
		}

		t2 = t1;

		BSpline<Eigen::Matrix<double, 3, 1>> spline(7);
		spline.setControl(cp, tt);

		BSpline<double> spline_yy(3);
		spline_yy.setControl(cpy, tt);

		visualization_msgs::Marker pose;
		Eigen::Matrix<double, 3, 1> ap = spline.evaluateCurve(0.0, 0);
		pose.header.frame_id = "map";
		pose.header.stamp = ros::Time::now();
		pose.ns = "ns";
		pose.id = 0;
		pose.type = visualization_msgs::Marker::SPHERE;
		pose.scale.x = 0.2;
		pose.scale.y = 0.2;
		pose.scale.z = 0.2;
		pose.pose.position.x = ap(0);
		pose.pose.position.y = ap(1);
		pose.pose.position.z = ap(2);
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		pose.color.a = 1.0;
		pose.color.r = 0.0;
		pose.color.g = 1.0;
		pose.color.b = 1.0;
		pose.action = visualization_msgs::Marker::ADD;
		visualPublisher_ps.publish(pose);

		pose.header.frame_id = "map";
		pose.header.stamp = ros::Time::now();
		pose.ns = "ns";
		pose.id = 1;
		pose.type = visualization_msgs::Marker::SPHERE;
		pose.scale.x = 0.2;
		pose.scale.y = 0.2;
		pose.scale.z = 0.2;
		pose.pose.position.x = xk(0);
		pose.pose.position.y = xk(1);
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		pose.color.a = 1.0;
		pose.color.r = 1.0;
		pose.color.g = 1.0;
		pose.color.b = 0.0;
		pose.action = visualization_msgs::Marker::ADD;
		visualPublisher_ps.publish(pose);

		std::vector<Eigen::Matrix<double, 3, 1>> traj;
		std::vector<double> traj_yy;
		for(double dt = 0.0; dt <= tt; dt += 0.05){
			traj.push_back(spline.evaluateCurve(dt, 0));
			traj_yy.push_back(spline_yy.evaluateCurve(dt, 0));
		}

		std::vector<Eigen::Matrix<double, 3, 1>> wp;
		std::vector<double> wp_yy;
		tracker.getPredictedRoverPose(wp, wp_yy);

		int id = 0;
		visualization_msgs::MarkerArray points;
		points.markers.resize(wp.size());
		for(uint ii = 0; ii < wp.size(); ii++){
			tf2::Quaternion q;
			q.setRPY(0.0, 0.0, wp_yy[ii]);

			points.markers[ii].header.frame_id = "map";
			points.markers[ii].header.stamp = ros::Time::now();
			points.markers[ii].ns = "ns";
			points.markers[ii].id = id++;
			points.markers[ii].type = visualization_msgs::Marker::ARROW;
			points.markers[ii].pose.position.x = wp[ii](0);
			points.markers[ii].pose.position.y = wp[ii](1);
			points.markers[ii].pose.position.z = wp[ii](2);
			points.markers[ii].pose.orientation.x = q.x();
			points.markers[ii].pose.orientation.y = q.y();
			points.markers[ii].pose.orientation.z = q.z();
			points.markers[ii].pose.orientation.w = q.w();
			points.markers[ii].scale.x = 0.5;
			points.markers[ii].scale.y = 0.1;
			points.markers[ii].scale.z = 0.1;
			points.markers[ii].color.a = 1.0;
			points.markers[ii].color.r = 0.0;
			points.markers[ii].color.g = 1.0;
			points.markers[ii].color.b = 0.0;
			points.markers[ii].action = visualization_msgs::Marker::ADD;
		}

		visualization_msgs::MarkerArray traj_pp;
		traj_pp.markers.resize(traj.size());
		for(uint ii = 0; ii < traj.size(); ii++){

			tf2::Quaternion q;
			q.setRPY(0.0, 0.0, traj_yy[ii]);

			traj_pp.markers[ii].header.frame_id = "map";
			traj_pp.markers[ii].header.stamp = ros::Time::now();
			traj_pp.markers[ii].ns = "ns";
			traj_pp.markers[ii].id = id++;
			traj_pp.markers[ii].type = visualization_msgs::Marker::ARROW;
			traj_pp.markers[ii].pose.position.x = traj[ii](0);
			traj_pp.markers[ii].pose.position.y = traj[ii](1);
			traj_pp.markers[ii].pose.position.z = traj[ii](2);
			traj_pp.markers[ii].pose.orientation.x = q.x();
			traj_pp.markers[ii].pose.orientation.y = q.y();
			traj_pp.markers[ii].pose.orientation.z = q.z();
			traj_pp.markers[ii].pose.orientation.w = q.w();
			traj_pp.markers[ii].scale.x = 0.5;
			traj_pp.markers[ii].scale.y = 0.1;
			traj_pp.markers[ii].scale.z = 0.1;
			traj_pp.markers[ii].color.a = 1.0;
			traj_pp.markers[ii].color.r = 1.0;
			traj_pp.markers[ii].color.g = 0.0;
			traj_pp.markers[ii].color.b = 0.0;
			traj_pp.markers[ii].action = visualization_msgs::Marker::ADD;
		}

		visualPublisher_tt.publish(traj_pp);
		visualPublisher_pp.publish(points);
		ros::spinOnce();
		rate.sleep();
	}

	delete environment;
	return 0;
}