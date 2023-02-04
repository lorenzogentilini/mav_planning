#include <mav_planning/planner.hpp>
#include <mav_planning/optimizer.hpp>
#include <mav_planning/containers.hpp>
#include <visualization_msgs/MarkerArray.h>

using namespace _Planner_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_planner");
	ros::NodeHandle nh("~");

	ros::Publisher visualPublisher_traj = nh.advertise<visualization_msgs::MarkerArray>("/trajectory", 1);
	ros::Publisher visualPublisher_vv = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_vv", 1);
	ros::Publisher visualPublisher_aa = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_aa", 1);
	ros::Publisher visualPublisher_jj = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_jj", 1);
	ros::Publisher visualPublisher_points = nh.advertise<visualization_msgs::MarkerArray>("/points", 1);

	Environment* environment = new Environment(nh);
	Planner planner(nh, environment);
	Optimizer optimizer(nh, environment);

	Eigen::Matrix<double, 6, 1> start;
	Eigen::Matrix<double, 6, 1> goal;

	start << -3.0, 9.0, 1.0, 0.0, 0.0, 0.0;
	goal << -20.0, 5.0, 1.0, 0.0, 0.0, 0.0;

	ros::Time t1 = ros::Time::now();
	ROS_INFO("[PLANNER TEST]: Start Planning");
	int result = planner.search(start, goal);
	if(result != GOAL_REACHED){
		ROS_ERROR("[PLANNER TEST]: Failed with %d", result);
		delete environment;
		exit(-1);
	}

	ros::Time t2 = ros::Time::now();
	ROS_INFO("[PLANNER TEST]: Executed in %f", (t2-t1).toSec());

	std::vector<Eigen::Matrix<double, 3, 1>> sampled_trajectory;
	double tt = planner.getTraj(sampled_trajectory, 0.5);
	ROS_INFO("[PLANNER TEST]: Sampled %ld Points", sampled_trajectory.size());

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> bounds;
	std::vector<Eigen::Matrix<double, 3, 1>> bounds_accumulator;
	Eigen::Matrix<double, 3, 1> zero = Eigen::MatrixXd::Zero(3, 1);

	bounds_accumulator.insert(bounds_accumulator.begin(), 2, zero);
	for(uint ii = 0; ii < 3; ii++){
		bounds.push_back(bounds_accumulator);
	}

	ROS_INFO("[PLANNER TEST]: Building Spline");
	BSpline<Eigen::Matrix<double, 3, 1>> trajectory(7);
	trajectory.setWaypoints(sampled_trajectory, bounds, tt);

	ROS_INFO("[PLANNER TEST]: Optimizing");
	t1 = ros::Time::now();
	std::vector<Eigen::Matrix<double, 3, 1>> cp = trajectory.getControl();
	optimizer.optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE);
	t2 = ros::Time::now();
	ROS_INFO("[PLANNER TEST]: Executed in %f", (t2-t1).toSec());

	ROS_INFO("[PLANNER TEST]: Building Spline");
	BSpline<Eigen::Matrix<double, 3, 1>> opt_trajectory(7);
	opt_trajectory.setControl(cp, tt);

	ROS_INFO("[PLANNER TEST]: Evaluation");
	std::vector<Eigen::Matrix<double, 3, 1>> evaluated_trajectory, evaluated_trajectory_opt;
	std::vector<Eigen::Matrix<double, 3, 1>> evaluated_trajectory_vv, evaluated_trajectory_vv_opt;
	std::vector<Eigen::Matrix<double, 3, 1>> evaluated_trajectory_aa, evaluated_trajectory_aa_opt;
	std::vector<Eigen::Matrix<double, 3, 1>> evaluated_trajectory_jj, evaluated_trajectory_jj_opt;
	for(double ta = 0.0; ta < tt; ta += 0.01){
		Eigen::Matrix<double, 3, 1> pp = trajectory.evaluateCurve(ta, 0);
		evaluated_trajectory.push_back(pp);

		Eigen::Matrix<double, 3, 1> pp_ = opt_trajectory.evaluateCurve(ta, 0);
		evaluated_trajectory_opt.push_back(pp_);

		Eigen::Matrix<double, 3, 1> vv = trajectory.evaluateCurve(ta, 1);
		evaluated_trajectory_vv.push_back(vv);

		Eigen::Matrix<double, 3, 1> vv_ = opt_trajectory.evaluateCurve(ta, 1);
		evaluated_trajectory_vv_opt.push_back(vv_);

		Eigen::Matrix<double, 3, 1> aa = trajectory.evaluateCurve(ta, 2);
		evaluated_trajectory_aa.push_back(aa);

		Eigen::Matrix<double, 3, 1> aa_ = opt_trajectory.evaluateCurve(ta, 2);
		evaluated_trajectory_aa_opt.push_back(aa_);

		Eigen::Matrix<double, 3, 1> jj = trajectory.evaluateCurve(ta, 3);
		evaluated_trajectory_jj.push_back(jj);

		Eigen::Matrix<double, 3, 1> jj_ = opt_trajectory.evaluateCurve(ta, 3);
		evaluated_trajectory_jj_opt.push_back(jj_);
	}

	// Publish Points
	visualization_msgs::MarkerArray voxelPoints;
	voxelPoints.markers.resize(sampled_trajectory.size());
	for(uint ii = 0; ii < voxelPoints.markers.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = sampled_trajectory[ii](0);
		pp.y = sampled_trajectory[ii](1);
		pp.z = sampled_trajectory[ii](2);

		voxelPoints.markers[ii].header.frame_id = "map";
		voxelPoints.markers[ii].header.stamp = ros::Time::now();
		voxelPoints.markers[ii].ns = "ns";
		voxelPoints.markers[ii].id = ii;
		voxelPoints.markers[ii].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelPoints.markers[ii].points.push_back(pp);
		voxelPoints.markers[ii].scale.x = 0.15;
		voxelPoints.markers[ii].scale.y = 0.15;
		voxelPoints.markers[ii].scale.z = 0.15;
		voxelPoints.markers[ii].color.a = 1.0;
		voxelPoints.markers[ii].color.r = 1.0;
		voxelPoints.markers[ii].color.g = 0.0;
		voxelPoints.markers[ii].color.b = 0.0;
		voxelPoints.markers[ii].action = visualization_msgs::Marker::ADD;
	}

	visualPublisher_points.publish(voxelPoints);

	// Publish Trajectory
	visualization_msgs::MarkerArray voxelTrajectory;
	visualization_msgs::MarkerArray voxelTrajectory_vv;
	visualization_msgs::MarkerArray voxelTrajectory_aa;
	visualization_msgs::MarkerArray voxelTrajectory_jj;
	voxelTrajectory.markers.resize(evaluated_trajectory.size() + evaluated_trajectory_opt.size());
	voxelTrajectory_vv.markers.resize(evaluated_trajectory.size() + evaluated_trajectory_opt.size());
	voxelTrajectory_aa.markers.resize(evaluated_trajectory.size() + evaluated_trajectory_opt.size());
	voxelTrajectory_jj.markers.resize(evaluated_trajectory.size() + evaluated_trajectory_opt.size());
	for(uint ii = 0; ii < evaluated_trajectory.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = evaluated_trajectory[ii](0);
		pp.y = evaluated_trajectory[ii](1);
		pp.z = evaluated_trajectory[ii](2);

		voxelTrajectory.markers[ii].header.frame_id = "map";
		voxelTrajectory.markers[ii].header.stamp = ros::Time::now();
		voxelTrajectory.markers[ii].ns = "ns";
		voxelTrajectory.markers[ii].id = ii;
		voxelTrajectory.markers[ii].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelTrajectory.markers[ii].points.push_back(pp);
		voxelTrajectory.markers[ii].scale.x = 0.05;
		voxelTrajectory.markers[ii].scale.y = 0.05;
		voxelTrajectory.markers[ii].scale.z = 0.05;
		voxelTrajectory.markers[ii].color.a = 1.0;
		voxelTrajectory.markers[ii].color.r = 0.0;
		voxelTrajectory.markers[ii].color.g = 1.0;
		voxelTrajectory.markers[ii].color.b = 0.0;
		voxelTrajectory.markers[ii].action = visualization_msgs::Marker::ADD;

		voxelTrajectory_vv.markers[ii] = voxelTrajectory.markers[ii];
		voxelTrajectory_aa.markers[ii] = voxelTrajectory.markers[ii];
		voxelTrajectory_jj.markers[ii] = voxelTrajectory.markers[ii];

		pp.x = evaluated_trajectory_vv[ii](0);
		pp.y = evaluated_trajectory_vv[ii](1);
		pp.z = evaluated_trajectory_vv[ii](2);
		voxelTrajectory_vv.markers[ii].points.clear();
		voxelTrajectory_vv.markers[ii].points.shrink_to_fit();
		voxelTrajectory_vv.markers[ii].points.push_back(pp);

		pp.x = evaluated_trajectory_aa[ii](0);
		pp.y = evaluated_trajectory_aa[ii](1);
		pp.z = evaluated_trajectory_aa[ii](2);
		voxelTrajectory_aa.markers[ii].points.clear();
		voxelTrajectory_aa.markers[ii].points.shrink_to_fit();
		voxelTrajectory_aa.markers[ii].points.push_back(pp);

		pp.x = evaluated_trajectory_jj[ii](0);
		pp.y = evaluated_trajectory_jj[ii](1);
		pp.z = evaluated_trajectory_jj[ii](2);
		voxelTrajectory_jj.markers[ii].points.clear();
		voxelTrajectory_jj.markers[ii].points.shrink_to_fit();
		voxelTrajectory_jj.markers[ii].points.push_back(pp);
	}

	int ss = evaluated_trajectory.size();
	for(uint ii = 0; ii < evaluated_trajectory_opt.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = evaluated_trajectory_opt[ii](0);
		pp.y = evaluated_trajectory_opt[ii](1);
		pp.z = evaluated_trajectory_opt[ii](2);

		voxelTrajectory.markers[ii + ss].header.frame_id = "map";
		voxelTrajectory.markers[ii + ss].header.stamp = ros::Time::now();
		voxelTrajectory.markers[ii + ss].ns = "ns";
		voxelTrajectory.markers[ii + ss].id = ii + ss;
		voxelTrajectory.markers[ii + ss].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelTrajectory.markers[ii + ss].points.push_back(pp);
		voxelTrajectory.markers[ii + ss].scale.x = 0.05;
		voxelTrajectory.markers[ii + ss].scale.y = 0.05;
		voxelTrajectory.markers[ii + ss].scale.z = 0.05;
		voxelTrajectory.markers[ii + ss].color.a = 1.0;
		voxelTrajectory.markers[ii + ss].color.r = 0.0;
		voxelTrajectory.markers[ii + ss].color.g = 1.0;
		voxelTrajectory.markers[ii + ss].color.b = 1.0;
		voxelTrajectory.markers[ii + ss].action = visualization_msgs::Marker::ADD;

		voxelTrajectory_vv.markers[ii + ss] = voxelTrajectory.markers[ii + ss];
		voxelTrajectory_aa.markers[ii + ss] = voxelTrajectory.markers[ii + ss];
		voxelTrajectory_jj.markers[ii + ss] = voxelTrajectory.markers[ii + ss];

		pp.x = evaluated_trajectory_vv_opt[ii + ss](0);
		pp.y = evaluated_trajectory_vv_opt[ii + ss](1);
		pp.z = evaluated_trajectory_vv_opt[ii + ss](2);
		voxelTrajectory_vv.markers[ii + ss].points.clear();
		voxelTrajectory_vv.markers[ii + ss].points.shrink_to_fit();
		voxelTrajectory_vv.markers[ii + ss].points.push_back(pp);

		pp.x = evaluated_trajectory_aa_opt[ii + ss](0);
		pp.y = evaluated_trajectory_aa_opt[ii + ss](1);
		pp.z = evaluated_trajectory_aa_opt[ii + ss](2);
		voxelTrajectory_aa.markers[ii + ss].points.clear();
		voxelTrajectory_aa.markers[ii + ss].points.shrink_to_fit();
		voxelTrajectory_aa.markers[ii + ss].points.push_back(pp);

		pp.x = evaluated_trajectory_jj_opt[ii + ss](0);
		pp.y = evaluated_trajectory_jj_opt[ii + ss](1);
		pp.z = evaluated_trajectory_jj_opt[ii + ss](2);
		voxelTrajectory_jj.markers[ii + ss].points.clear();
		voxelTrajectory_jj.markers[ii + ss].points.shrink_to_fit();
		voxelTrajectory_jj.markers[ii + ss].points.push_back(pp);
	}

	visualPublisher_traj.publish(voxelTrajectory);
	visualPublisher_vv.publish(voxelTrajectory_vv);
	visualPublisher_aa.publish(voxelTrajectory_aa);
	visualPublisher_jj.publish(voxelTrajectory_jj);

	ros::spin();

	delete environment;
	return 0;
}