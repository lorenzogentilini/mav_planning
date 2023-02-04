#pragma once
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <vector>
#include <thread>

#include <mav_planning/containers.hpp>
#include <mav_planning/environment.hpp>
#include <mav_planning/optimizer.hpp>

namespace _Replanner_{
class GraphNode{
	public:
	enum NODE_TYPE { GUARD = 1, CONNECTOR = 2 };
  	enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

	std::vector<GraphNode*> neighbors;
	Eigen::Matrix<double, 3, 1> pp;
	NODE_TYPE type;
	NODE_STATE state;
	int id;

	// Class Constructor
	GraphNode(Eigen::Matrix<double, 3, 1> pp_, NODE_TYPE type_, int id_){
		pp = pp_;
		type = type_;
		id = id_;
	}

	// Class Destructor
	~GraphNode(){
		;
	}
};

class Replanner{
	private:
	bool debug_flag, multi_thrs, do_final_opt;
	bool is_prev_collision = false;

	double dt_max, dt_res, dt_min;
	double safe_margin, obs_size;
	double max_sample_time, m_res;

	double m_max_x, m_max_y, m_max_z;
	double m_min_x, m_min_y, m_min_z;

	int max_sample_pts, max_pts_nn;
	int max_opt_paths_nn;

	Eigen::Matrix<double, 3, 1> si;
	std::vector<GraphNode*> graph;
	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> paths;

	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	Environment* environment;
	std::vector<Optimizer*> optimizer;
	BSpline<Eigen::Matrix<double, 3, 1>>* spline;
	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> cps;

	double getDistance(double x, double y, double z);
	void createGraph(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf);
	Eigen::Matrix<double, 3, 1> samplePoint(Eigen::Matrix<double, 3, 1> bb, Eigen::Matrix<double, 3, 1> tt, Eigen::Matrix<double, 3, 3> rr);
	std::vector<GraphNode*> findVisibleGuards(Eigen::Matrix<double, 3, 1> pp);
	bool isVisible(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf, double trh);
	bool needConnection(GraphNode* ni, GraphNode* nf, Eigen::Matrix<double, 3, 1> pp);
	double pathLength(std::vector<Eigen::Matrix<double, 3, 1>> pt);
	bool samePath(std::vector<Eigen::Matrix<double, 3, 1>> pt_1, std::vector<Eigen::Matrix<double, 3, 1>> pt_2);
	std::vector<Eigen::Matrix<double, 3, 1>> discretizePath(std::vector<Eigen::Matrix<double, 3, 1>> pt);
	std::vector<Eigen::Matrix<double, 3, 1>> discretizePath(std::vector<Eigen::Matrix<double, 3, 1>> pt, int nn);
	void pruneGraph();
	void extractPaths();
	void depthSearch(std::vector<GraphNode*>& visited);
	void shortcutPaths();
	void shortcutPath(std::vector<Eigen::Matrix<double, 3, 1>> pt, int idx);
	void pruneEquivalent();
	void sortPaths();
	void sortTrajectories();
	bool insideBounds(Eigen::Matrix<double, 3, 1> pp);
	void optimizePath(std::vector<Eigen::Matrix<double, 3, 1>> pt, double ti, double tf, int idx);
	static bool sortByLenght(std::vector<Eigen::Matrix<double, 3, 1>> pt1, std::vector<Eigen::Matrix<double, 3, 1>> pt2);
	static bool sortByCost(std::vector<Eigen::Matrix<double, 3, 1>> cp1, std::vector<Eigen::Matrix<double, 3, 1>> cp2);

	public:
	// Class Constructor
	Replanner(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Replanner();

	static const int NO_COLLISION = (1 << 0);
  	static const int COLLISION = (1 << 1);
  	static const int EMERGENCY_COLLISION = (1 << 2);

	void setInflation(Eigen::Matrix<double, 3, 1> ii);
	void setTrajectory(std::vector<Eigen::Matrix<double, 3, 1>> cp, double tt);
	void getTrajectory(std::vector<Eigen::Matrix<double, 3, 1>>& cp, double& tt);
	void getTrajectories(std::vector<std::vector<Eigen::Matrix<double, 3, 1>>>& _cps);
	void getPaths(std::vector<std::vector<Eigen::Matrix<double, 3, 1>>>& pts);
	void findTopologicalPaths(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf);
	bool replanTrajectory(double ti, double tc);
	int superviseTrajectory(double tt);
};
}