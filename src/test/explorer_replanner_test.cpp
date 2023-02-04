#include <mav_planning/replanner.hpp>
#include <mav_planning/explorer.hpp>
#include <mav_planning/containers.hpp>

using namespace _Explorer_;
using namespace _Replanner_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_explorer_replanner");
	ros::NodeHandle nh("~");

	ros::Publisher visualPublisher_tt = nh.advertise<visualization_msgs::MarkerArray>("/trajs", 1);

	Environment* environment = new Environment(nh);
	Replanner replanner(nh, environment);
	Explorer explorer(nh, environment);

	Eigen::Matrix<double, 2, 1> pose = Eigen::Matrix<double, 2, 1> (-12, 9);

	ros::Time t1 = ros::Time::now();
	std::vector<Eigen::Matrix<double, 3, 1>> wp;
	explorer.explore(pose, wp);
	ros::Time t2 = ros::Time::now();
	ROS_INFO("[EXPLORER-REPLANNER TEST]: Executed in %f", (t2-t1).toSec());

	ROS_INFO("[EXPLORER-REPLANNER TEST]: Get BSpline");
	t1 = ros::Time::now();
	double tt;
	std::vector<Eigen::Matrix<double, 3, 1>> cp;
	explorer.fitWithBSpline(cp, tt);
	t2 = ros::Time::now();
	ROS_INFO("[EXPLORER-REPLANNER TEST]: Done in %f", (t2-t1).toSec());

	replanner.setTrajectory(cp, tt);
	
	ROS_INFO("[EXPLORER-REPLANNER TEST]: Start Replanning");
	t1 = ros::Time::now();
	if(!replanner.replanTrajectory(20.0, 22.0)){
		ROS_ERROR("[EXPLORER-REPLANNER TEST]: Replanning Went Wrong");
	}
	t2 = ros::Time::now();
	ROS_INFO("[EXPLORER-REPLANNER TEST]: Replanning Done in %f", (t2-t1).toSec());

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> cps;
	replanner.getTrajectories(cps);
	cps.push_back(cp);
	
	std::vector<Eigen::Matrix<double, 3, 1>> best_cp;

	double total_tt = 0.0;
	replanner.getTrajectory(best_cp, total_tt);
	cps.push_back(best_cp);

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> pts;
	replanner.getPaths(pts);
	ROS_INFO("[EXPLORER-REPLANNER TEST]: Get %ld Paths", pts.size());

	// Publish Points
	int id = 0;
	BSpline<Eigen::Matrix<double, 3, 1>> spline(7);
	visualization_msgs::MarkerArray voxelPoints_tt;
	for(uint ii = 0; ii < cps.size(); ii++){
		std::vector<Eigen::Matrix<double, 3, 1>> traj;
		spline.setControl(cps[ii], total_tt);
		for(double ts = 0.0; ts <= total_tt; ts += 0.1){
			traj.push_back(spline.evaluateCurve(ts, 0));
		}

		for(uint hh = 0; hh < traj.size(); hh++){
			geometry_msgs::Point pp;
			pp.x = traj[hh](0);
			pp.y = traj[hh](1);
			pp.z = traj[hh](2);

			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "ns";
			marker.id = id++;
			marker.type = visualization_msgs::Marker::SPHERE_LIST;
			marker.points.push_back(pp);
			marker.scale.x = 0.05;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			if(ii == cps.size()-1){
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
			}

			if(ii == cps.size()-2){
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
			}

			marker.action = visualization_msgs::Marker::ADD;
			voxelPoints_tt.markers.push_back(marker);
		}
	}

	visualPublisher_tt.publish(voxelPoints_tt);

	ros::spin();

	delete environment;
	return 0;
}