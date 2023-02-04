#include <mav_planning/replanner.hpp>
#include <mav_planning/containers.hpp>

using namespace _Replanner_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_replanner");
	ros::NodeHandle nh("~");

	ros::Publisher visualPublisher_pp = nh.advertise<visualization_msgs::MarkerArray>("/paths", 1);
	ros::Publisher visualPublisher_tt = nh.advertise<visualization_msgs::MarkerArray>("/trajs", 1);
	ros::Publisher visualPublisher_vv = nh.advertise<visualization_msgs::MarkerArray>("/vel_trajectory", 1);
	ros::Publisher visualPublisher_aa = nh.advertise<visualization_msgs::MarkerArray>("/acc_trajectory", 1);
	ros::Publisher visualPublisher_jj = nh.advertise<visualization_msgs::MarkerArray>("/jerk_trajectory", 1);

	Environment* environment = new Environment(nh);
	Replanner replanner(nh, environment);
	BSpline<Eigen::Matrix<double, 3, 1>> spline(7);

	double tt = 20.0;
	Eigen::Matrix<double, 3, 1> pi = Eigen::Matrix<double, 3, 1>(-2.5, 5.5, 1.6);
	Eigen::Matrix<double, 3, 1> pw1 = Eigen::Matrix<double, 3, 1>(-5.5, 3.5, 1.6);
	Eigen::Matrix<double, 3, 1> pw2 = Eigen::Matrix<double, 3, 1>(-15.0, 3.5, 1.6);
	Eigen::Matrix<double, 3, 1> pf = Eigen::Matrix<double, 3, 1>(-17.0, 5.5, 1.6);
	std::vector<Eigen::Matrix<double, 3, 1>> cp;
	cp.insert(cp.end(), 4, pi);
	cp.insert(cp.end(), 2, pw1);
	cp.insert(cp.end(), 2, pw2);
	cp.insert(cp.end(), 4, pf);
	replanner.setTrajectory(cp, tt);
	spline.setControl(cp, tt);
	
	ROS_INFO("[REPLANNER TEST]: Start Replanning");
	ros::Time t1 = ros::Time::now();
	if(!replanner.replanTrajectory(8.0, 10.0)){
		ROS_ERROR("[REPLANNER TEST]: Replanning Went Wrong");
	}

	ros::Time t2 = ros::Time::now();
	ROS_INFO("[REPLANNER TEST]: Replanning Done in %f", (t2-t1).toSec());

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> pts;
	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> cps;

	replanner.getPaths(pts);
	replanner.getTrajectories(cps);

	cps.push_back(cp);

	std::vector<Eigen::Matrix<double, 3, 1>> best_cp;
	double total_tt = 0.0;
	replanner.getTrajectory(best_cp, total_tt);
	cps.push_back(best_cp);

	ROS_INFO("[REPLANNER TEST]: Get %ld Paths", pts.size());

	for(uint ii = 0; ii < best_cp.size(); ii++){
		ROS_INFO("[REPLANNER TEST]: Get %f %f %f", best_cp[ii](0), best_cp[ii](1), best_cp[ii](2));
	}

	std::vector<Eigen::Matrix<double, 3, 1>> big_points;
	std::vector<Eigen::Matrix<double, 3, 1>> points;
	for(uint ii = 0; ii < pts.size(); ii++){
		for(uint jj = 0; jj < pts[ii].size()-1; jj++){
			big_points.push_back(pts[ii][jj]);
			Eigen::Matrix<double, 3, 1> pi = pts[ii][jj];
			Eigen::Matrix<double, 3, 1> pf = pts[ii][jj+1];

			double rr = (pi-pf).norm();
			Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();
			for(double dr = 0.0; dr < rr; dr += 0.1){
				Eigen::Matrix<double, 3, 1> pp = pi + vv*dr;
				points.push_back(pp);
			}
		}

		big_points.push_back(pts[ii][pts[ii].size()-1]);
	}

	// Publish Points
	visualization_msgs::MarkerArray voxelPoints_pp;
	voxelPoints_pp.markers.resize(big_points.size()+points.size());
	for(uint ii = 0; ii < big_points.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = big_points[ii](0);
		pp.y = big_points[ii](1);
		pp.z = big_points[ii](2);

		voxelPoints_pp.markers[ii].header.frame_id = "map";
		voxelPoints_pp.markers[ii].header.stamp = ros::Time::now();
		voxelPoints_pp.markers[ii].ns = "ns";
		voxelPoints_pp.markers[ii].id = ii;
		voxelPoints_pp.markers[ii].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelPoints_pp.markers[ii].points.push_back(pp);
		voxelPoints_pp.markers[ii].scale.x = 0.2;
		voxelPoints_pp.markers[ii].scale.y = 0.2;
		voxelPoints_pp.markers[ii].scale.z = 0.2;
		voxelPoints_pp.markers[ii].color.a = 1.0;
		voxelPoints_pp.markers[ii].color.r = 1.0;
		voxelPoints_pp.markers[ii].color.g = 0.0;
		voxelPoints_pp.markers[ii].color.b = 0.0;
		voxelPoints_pp.markers[ii].action = visualization_msgs::Marker::ADD;
	}

	for(uint ii = 0; ii < points.size(); ii++){
		geometry_msgs::Point pp;
		pp.x = points[ii](0);
		pp.y = points[ii](1);
		pp.z = points[ii](2);

		voxelPoints_pp.markers[ii+big_points.size()].header.frame_id = "map";
		voxelPoints_pp.markers[ii+big_points.size()].header.stamp = ros::Time::now();
		voxelPoints_pp.markers[ii+big_points.size()].ns = "ns";
		voxelPoints_pp.markers[ii+big_points.size()].id = ii+big_points.size();
		voxelPoints_pp.markers[ii+big_points.size()].type = visualization_msgs::Marker::SPHERE_LIST;
		voxelPoints_pp.markers[ii+big_points.size()].points.push_back(pp);
		voxelPoints_pp.markers[ii+big_points.size()].scale.x = 0.05;
		voxelPoints_pp.markers[ii+big_points.size()].scale.y = 0.05;
		voxelPoints_pp.markers[ii+big_points.size()].scale.z = 0.05;
		voxelPoints_pp.markers[ii+big_points.size()].color.a = 1.0;
		voxelPoints_pp.markers[ii+big_points.size()].color.r = 0.0;
		voxelPoints_pp.markers[ii+big_points.size()].color.g = 1.0;
		voxelPoints_pp.markers[ii+big_points.size()].color.b = 0.0;
		voxelPoints_pp.markers[ii+big_points.size()].action = visualization_msgs::Marker::ADD;
	}

	int id = 0;
	visualization_msgs::MarkerArray voxelPoints_tt;
	for(uint ii = 0; ii < cps.size(); ii++){
		std::vector<Eigen::Matrix<double, 3, 1>> traj;
		std::vector<Eigen::Matrix<double, 3, 1>> vv;
		std::vector<Eigen::Matrix<double, 3, 1>> aa;
		std::vector<Eigen::Matrix<double, 3, 1>> jj;

		visualization_msgs::MarkerArray voxelPoints_vv;
		visualization_msgs::MarkerArray voxelPoints_aa;
		visualization_msgs::MarkerArray voxelPoints_jj;

		if (ii == cps.size()-2){
			spline.setControl(cps[ii], tt);
			for(double ts = 0.0; ts <= tt; ts += 0.1){
				traj.push_back(spline.evaluateCurve(ts, 0));
				vv.push_back(spline.evaluateCurve(ts, 1));
				aa.push_back(spline.evaluateCurve(ts, 2));
				jj.push_back(spline.evaluateCurve(ts, 3));
			}
		} else{
			spline.setControl(cps[ii], total_tt);
			for(double ts = 0.0; ts <= total_tt; ts += 0.1){
				traj.push_back(spline.evaluateCurve(ts, 0));
				vv.push_back(spline.evaluateCurve(ts, 1));
				aa.push_back(spline.evaluateCurve(ts, 2));
				jj.push_back(spline.evaluateCurve(ts, 3));
			}
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

			marker.action = visualization_msgs::Marker::ADD;
			voxelPoints_tt.markers.push_back(marker);

			pp.x = vv[hh](0);
			pp.y = vv[hh](1);
			pp.z = vv[hh](2);
			marker.points.clear();
			marker.points.shrink_to_fit();
			marker.points.push_back(pp);

			voxelPoints_vv.markers.push_back(marker);

			pp.x = aa[hh](0);
			pp.y = aa[hh](1);
			pp.z = aa[hh](2);
			marker.points.clear();
			marker.points.shrink_to_fit();
			marker.points.push_back(pp);

			voxelPoints_aa.markers.push_back(marker);

			pp.x = jj[hh](0);
			pp.y = jj[hh](1);
			pp.z = jj[hh](2);
			marker.points.clear();
			marker.points.shrink_to_fit();
			marker.points.push_back(pp);

			voxelPoints_jj.markers.push_back(marker);
		}

		visualPublisher_vv.publish(voxelPoints_vv);
		visualPublisher_aa.publish(voxelPoints_aa);
		visualPublisher_jj.publish(voxelPoints_jj);
	}

	visualPublisher_pp.publish(voxelPoints_pp);
	visualPublisher_tt.publish(voxelPoints_tt);

	ros::spin();

	delete environment;
	return 0;
}