#include <mav_planning/explorer.hpp>
#include <mav_planning/containers.hpp>

using namespace _Explorer_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_explorer");
	ros::NodeHandle nh("~");

	ros::Publisher visualPublisher = nh.advertise<visualization_msgs::MarkerArray>("/trajectory", 1);

	Environment* environment = new Environment(nh);
	Explorer* explorer = new Explorer(nh, environment);

	Eigen::Matrix<double, 2, 1> pose;
	pose << -11.5, 8.5;

	std::vector<Eigen::Matrix<double, 3, 1>> traj_points;
	std::vector<Eigen::Matrix<double, 3, 1>> points;
	std::vector<Eigen::Matrix<double, 3, 1>> wp;

	for(uint ii = 0; ii < 5; ii++){
		if(ii != 0){
			Eigen::Matrix<double, 3, 1> pf = traj_points.back();
			pose(0) = pf(0);
			pose(1) = pf(1);
		}

		ros::Time t1 = ros::Time::now();
		std::vector<Eigen::Matrix<double, 3, 1>> wpi;
		explorer->explore(pose, wpi);

		wp.insert(wp.end(), wpi.begin(), wpi.end());
		ros::Time t2 = ros::Time::now();
		ROS_INFO("[EXPLORER TEST]: Executed in %f", (t2-t1).toSec());

		ROS_INFO("[EXPLORER TEST]: Get BSpline");
		t1 = ros::Time::now();
		double tt;
		std::vector<Eigen::Matrix<double, 3, 1>> cp;
		explorer->fitWithBSpline(cp, tt);
		t2 = ros::Time::now();
		ROS_INFO("[EXPLORER TEST]: Done in %f", (t2-t1).toSec());

		BSpline<Eigen::Matrix<double, 3, 1>> spline(7);
		spline.setControl(cp, tt);

		for(uint ii = 0; ii < wpi.size()-1; ii++){
			Eigen::Matrix<double, 3, 1> pi = wpi[ii];
			Eigen::Matrix<double, 3, 1> pf = wpi[ii+1];

			double rr = (pi-pf).norm();
			Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();
			for(double dr = 0.0; dr < rr; dr += 0.1){
				Eigen::Matrix<double, 3, 1> pp = pi + vv*dr;
				points.push_back(pp);
			}
		}

		for(double dt = 0.0; dt < tt; dt += 0.2){
			Eigen::Matrix<double, 3, 1> pp = spline.evaluateCurve(dt, 0);
			traj_points.push_back(pp);
		}
	}

	ROS_INFO("[EXPLORER TEST]: Size %ld", traj_points.size());

	// Publish Points
	visualization_msgs::MarkerArray voxelPoints;
	voxelPoints.markers.resize(wp.size()+points.size()+traj_points.size());
	for(uint ii = 0; ii < wp.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = wp[ii](0);
		pp.y = wp[ii](1);
		pp.z = wp[ii](2);

		voxelPoints.markers[ii].header.frame_id = "map";
		voxelPoints.markers[ii].header.stamp = ros::Time::now();
		voxelPoints.markers[ii].ns = "ns";
		voxelPoints.markers[ii].id = ii;
		voxelPoints.markers[ii].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelPoints.markers[ii].points.push_back(pp);
		voxelPoints.markers[ii].scale.x = 0.1;
		voxelPoints.markers[ii].scale.y = 0.1;
		voxelPoints.markers[ii].scale.z = 0.1;
		voxelPoints.markers[ii].color.a = 1.0;
		voxelPoints.markers[ii].color.r = 1.0;
		voxelPoints.markers[ii].color.g = 0.0;
		voxelPoints.markers[ii].color.b = 0.0;
		voxelPoints.markers[ii].action = visualization_msgs::Marker::ADD;
	}

	for(uint ii = 0; ii < points.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = points[ii](0);
		pp.y = points[ii](1);
		pp.z = points[ii](2);

		voxelPoints.markers[ii+wp.size()].header.frame_id = "map";
		voxelPoints.markers[ii+wp.size()].header.stamp = ros::Time::now();
		voxelPoints.markers[ii+wp.size()].ns = "ns";
		voxelPoints.markers[ii+wp.size()].id = ii+wp.size();
		voxelPoints.markers[ii+wp.size()].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelPoints.markers[ii+wp.size()].points.push_back(pp);
		voxelPoints.markers[ii+wp.size()].scale.x = 0.05;
		voxelPoints.markers[ii+wp.size()].scale.y = 0.05;
		voxelPoints.markers[ii+wp.size()].scale.z = 0.05;
		voxelPoints.markers[ii+wp.size()].color.a = 1.0;
		voxelPoints.markers[ii+wp.size()].color.r = 0.0;
		voxelPoints.markers[ii+wp.size()].color.g = 1.0;
		voxelPoints.markers[ii+wp.size()].color.b = 0.0;
		voxelPoints.markers[ii+wp.size()].action = visualization_msgs::Marker::ADD;
	}

	for(uint ii = 0; ii < traj_points.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = traj_points[ii](0);
		pp.y = traj_points[ii](1);
		pp.z = traj_points[ii](2);

		voxelPoints.markers[ii+wp.size()+points.size()].header.frame_id = "map";
		voxelPoints.markers[ii+wp.size()+points.size()].header.stamp = ros::Time::now();
		voxelPoints.markers[ii+wp.size()+points.size()].ns = "ns";
		voxelPoints.markers[ii+wp.size()+points.size()].id = ii+wp.size()+points.size();
		voxelPoints.markers[ii+wp.size()+points.size()].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelPoints.markers[ii+wp.size()+points.size()].points.push_back(pp);
		voxelPoints.markers[ii+wp.size()+points.size()].scale.x = 0.1;
		voxelPoints.markers[ii+wp.size()+points.size()].scale.y = 0.1;
		voxelPoints.markers[ii+wp.size()+points.size()].scale.z = 0.1;
		voxelPoints.markers[ii+wp.size()+points.size()].color.a = 1.0;
		voxelPoints.markers[ii+wp.size()+points.size()].color.r = 1.0;
		voxelPoints.markers[ii+wp.size()+points.size()].color.g = 2.0;
		voxelPoints.markers[ii+wp.size()+points.size()].color.b = 0.0;
		voxelPoints.markers[ii+wp.size()+points.size()].action = visualization_msgs::Marker::ADD;
	}

	visualPublisher.publish(voxelPoints);

	ros::spin();

	delete environment;
	delete explorer;
	
	return 0;
}