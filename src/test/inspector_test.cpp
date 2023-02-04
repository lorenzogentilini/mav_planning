#include <mav_planning/inspector.hpp>
#include <mav_planning/containers.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace _Inspector_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_inspector");
	ros::NodeHandle nh("~");

	ros::Publisher visualPublisher_pp = nh.advertise<visualization_msgs::MarkerArray>("/points", 1);
	ros::Publisher visualPublisher_pt = nh.advertise<visualization_msgs::MarkerArray>("/paths", 1);
	ros::Publisher visualPublisher_tt = nh.advertise<visualization_msgs::MarkerArray>("/traj", 1);

	Environment* environment = new Environment(nh);
	Inspector inspector(nh, environment);

	std::vector<Eigen::Matrix<double, 2, 1>> ips;
	std::vector<Eigen::Matrix<double, 3, 1>> points;
	std::vector<Eigen::Matrix<double, 3, 1>> points_pt;
	std::vector<Eigen::Matrix<double, 7, 1>> points_tt;

	// Points to Be Inspected
	Eigen::Matrix<double, 2, 1> pp;

	pp << -10.5, 7.5;
	ips.push_back(pp);

	pp << -2, 8.5;
	ips.push_back(pp);

	pp << -4, 6;
	ips.push_back(pp);

	pp << -12, 3.5;
	ips.push_back(pp);

	pp << -6, 1;
	ips.push_back(pp);

	// Start Inspection
	for(uint ii = 0; ii < ips.size(); ii++){
		ROS_INFO("[INSPECTOR TEST]: Start Inspection");
		ros::Time t1 = ros::Time::now();
		inspector.inspect(ips[ii]);
		ros::Time t2 = ros::Time::now();
		ROS_INFO("[INSPECTOR TEST]: End Inspection, Elapsed in %f", (t2-t1).toSec());

		std::vector<Eigen::Matrix<double, 3, 1>> pps;
		inspector.getInspectPoints(pps);

		std::vector<Eigen::Matrix<double, 3, 1>> pth;
		inspector.getPath(pth);

		double tt;
		std::vector<Eigen::Matrix<double, 3, 1>> cp;
		std::vector<double> cpy;
		inspector.getTrajectory(cp, cpy, tt);
		ROS_INFO("[INSPECTOR TEST]: Get %ld Points", pps.size());

		points.insert(points.end(), pps.begin(), pps.end());

		if(pth.size() != 0){
			for(uint jj = 0; jj < pth.size()-1; jj++){
				double rr = (pth[jj]-pth[jj+1]).norm();
				Eigen::Matrix<double, 3, 1> vv = (pth[jj+1]-pth[jj]).normalized();

				for(double dr = 0.0; dr < rr; dr += 0.05){
					Eigen::Matrix<double, 3, 1> ppi = pth[jj] + vv*dr;
					points_pt.push_back(ppi);
				}
			}
		}

		if(cp.size() != 0){
			ROS_INFO("[INSPECTOR TEST]: Get %ld Control Points", cp.size());

			BSpline<Eigen::Matrix<double, 3, 1>> curve(7);
			curve.setControl(cp, tt);

			BSpline<double> curve_y(3);
			curve_y.setControl(cpy, tt);

			for(double dt = 0.0; dt < tt; dt += 0.1){
				t1 = ros::Time::now();
				Eigen::Matrix<double, 3, 1> pi = curve.evaluateCurve(dt, 0);
				t2 = ros::Time::now();

				double yi = curve_y.evaluateCurve(dt, 0);
				tf2::Quaternion qq;
				qq.setRPY(0.0, 0.0, yi);

				Eigen::Matrix<double, 7, 1> ti;
				ti(0) = pi(0);
				ti(1) = pi(1);
				ti(2) = pi(2);
				ti(3) = qq.x();
				ti(4) = qq.y();
				ti(5) = qq.z();
				ti(6) = qq.w();

				points_tt.push_back(ti);
			}

			ROS_INFO("[INSPECTOR TEST]: Curve Evaluation Time %f", (t2-t1).toSec());
		}
	}

	int id = 0;
	visualization_msgs::MarkerArray vis_points;
	vis_points.markers.resize(points.size());
	for(uint ii = 0; ii < points.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = points[ii](0);
		pp.y = points[ii](1);
		pp.z = points[ii](2);

		vis_points.markers[ii].header.frame_id = "map";
		vis_points.markers[ii].header.stamp = ros::Time::now();
		vis_points.markers[ii].ns = "ns";
		vis_points.markers[ii].id = id++;
		vis_points.markers[ii].type = visualization_msgs::Marker::SPHERE_LIST;
		vis_points.markers[ii].points.push_back(pp);
		vis_points.markers[ii].scale.x = 0.1;
		vis_points.markers[ii].scale.y = 0.1;
		vis_points.markers[ii].scale.z = 0.1;
		vis_points.markers[ii].color.a = 1.0;
		vis_points.markers[ii].color.r = 1.0;
		vis_points.markers[ii].color.g = 0.0;
		vis_points.markers[ii].color.b = 0.0;
		vis_points.markers[ii].action = visualization_msgs::Marker::ADD;
	}

	visualization_msgs::MarkerArray vis_pts;
	vis_pts.markers.resize(points_pt.size());
	for(uint ii = 0; ii < points_pt.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = points_pt[ii](0);
		pp.y = points_pt[ii](1);
		pp.z = points_pt[ii](2);

		vis_pts.markers[ii].header.frame_id = "map";
		vis_pts.markers[ii].header.stamp = ros::Time::now();
		vis_pts.markers[ii].ns = "ns";
		vis_pts.markers[ii].id = id++;
		vis_pts.markers[ii].type = visualization_msgs::Marker::SPHERE_LIST;
		vis_pts.markers[ii].points.push_back(pp);
		vis_pts.markers[ii].scale.x = 0.05;
		vis_pts.markers[ii].scale.y = 0.05;
		vis_pts.markers[ii].scale.z = 0.05;
		vis_pts.markers[ii].color.a = 1.0;
		vis_pts.markers[ii].color.r = 0.0;
		vis_pts.markers[ii].color.g = 1.0;
		vis_pts.markers[ii].color.b = 0.0;
		vis_pts.markers[ii].action = visualization_msgs::Marker::ADD;
	}

	visualization_msgs::MarkerArray vis_traj;
	vis_traj.markers.resize(points_tt.size());
	for(uint ii = 0; ii < points_tt.size(); ii++){
		vis_traj.markers[ii].header.frame_id = "map";
		vis_traj.markers[ii].header.stamp = ros::Time::now();
		vis_traj.markers[ii].ns = "ns";
		vis_traj.markers[ii].id = id++;
		vis_traj.markers[ii].type = visualization_msgs::Marker::ARROW;

		vis_traj.markers[ii].pose.position.x = points_tt[ii](0);
		vis_traj.markers[ii].pose.position.y = points_tt[ii](1);
		vis_traj.markers[ii].pose.position.z = points_tt[ii](2);

		vis_traj.markers[ii].pose.orientation.x = points_tt[ii](3);
		vis_traj.markers[ii].pose.orientation.y = points_tt[ii](4);
		vis_traj.markers[ii].pose.orientation.z = points_tt[ii](5);
		vis_traj.markers[ii].pose.orientation.w = points_tt[ii](6);

		vis_traj.markers[ii].scale.x = 0.2;
		vis_traj.markers[ii].scale.y = 0.02;
		vis_traj.markers[ii].scale.z = 0.02;
		vis_traj.markers[ii].color.a = 1.0;
		vis_traj.markers[ii].color.r = 1.0;
		vis_traj.markers[ii].color.g = 1.0;
		vis_traj.markers[ii].color.b = 0.0;
		vis_traj.markers[ii].action = visualization_msgs::Marker::ADD;
	}

	visualPublisher_pt.publish(vis_pts);
	visualPublisher_pp.publish(vis_points);
	visualPublisher_tt.publish(vis_traj);
	ros::spin();

	delete environment;
	return 0;
}