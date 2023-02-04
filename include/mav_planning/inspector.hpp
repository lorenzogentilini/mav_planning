#pragma once
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <vector>
#include <thread>

#include <mav_planning/environment.hpp>
#include <mav_planning/optimizer.hpp>
#include <mav_planning/containers.hpp>

namespace _Inspector_{
class Node{
	public:
	Eigen::Matrix<double, 3, 1> pp;
	Eigen::Matrix<double, 3, 1> cc;
	int id;

	Node(){;};
};

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

class Inspector{
	private:
	double opt_rr, delta_th, delta_zz;
	double max_rr, mm_res, safe_margin;
	double m_max_x, m_max_y, m_max_z;
	double m_min_x, m_min_y, m_min_z;
	double up_lr, up_nn, init_lr;
	double ff_relax, tt, max_tt;
	double smooth_cc, guide_cc, max_vv;

	int nn_max, id, max_iter, max_nn_pts;

	bool debug_flag, use_last_pp;

	Eigen::Matrix<double, 3, 1> si;
	std::vector<Node> vp_graph;
	std::vector<GraphNode*> ll_graph;
	std::vector<Eigen::Matrix<double, 3, 1>> pth;
	std::vector<Eigen::Matrix<double, 3, 1>> cp;
	std::vector<double> cpy;
	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> paths;

	Environment* environment;
	Optimizer* optimizer;

	void clearAll();
	void generateViewpoints(Eigen::Matrix<double, 2, 1> pp);
	void solveTSP();
	void correctPath();
	void computeTrajectory(Eigen::Matrix<double, 2, 1> pp);
	void shortcutPaths();
	void extractPaths();
	void pruneGraph();
	void shortcutPath(std::vector<Eigen::Matrix<double, 3, 1>> pt, int idx);
	bool generateLayerPoints(std::vector<Eigen::Matrix<double, 3, 1>>& pps, Eigen::Matrix<double, 3, 1> pp);
	bool isLineVisible(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf, double thr);
	bool isFeasible(Eigen::Matrix<double, 3, 1> pp);
	bool isInsideBounds(Eigen::Matrix<double, 3, 1> pp);
	void computePath(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf);
	void depthSearch(std::vector<GraphNode*>& visited);
	double pathLength(std::vector<Eigen::Matrix<double, 3, 1>> pt);
	bool needConnection(GraphNode* ni, GraphNode* nf, Eigen::Matrix<double, 3, 1> pp);
	bool samePath(std::vector<Eigen::Matrix<double, 3, 1>> pt_1, std::vector<Eigen::Matrix<double, 3, 1>> pt_2);
	std::vector<GraphNode*> findVisibleGuards(Eigen::Matrix<double, 3, 1> pp);
	std::vector<Eigen::Matrix<double, 3, 1>> discretizePath_nn(std::vector<Eigen::Matrix<double, 3, 1>> pt, int nn);
	std::vector<Eigen::Matrix<double, 3, 1>> discretizePath_dd(std::vector<Eigen::Matrix<double, 3, 1>> pt, double dd);

	Eigen::Matrix<double, 3, 1> samplePoint(double th_mn, double th_mx, Eigen::Matrix<double, 3, 1> pa);
	Eigen::Matrix<double, 3, 1> samplePoint(Eigen::Matrix<double, 3, 1> bb,
												   Eigen::Matrix<double, 3, 1> tt,
												   Eigen::Matrix<double, 3, 3> rr);

	static bool sortResult(Node n1, Node n2){ return n1.id < n2.id;  };
	static bool sortByLenght(std::vector<Eigen::Matrix<double, 3, 1>> pt1, std::vector<Eigen::Matrix<double, 3, 1>> pt2);

	inline double dx(Eigen::Matrix<double, 3, 1> p1, Eigen::Matrix<double, 2, 1> p2){ return p2(0)-p1(0); };
	inline double dy(Eigen::Matrix<double, 3, 1> p1, Eigen::Matrix<double, 2, 1> p2){ return p2(1)-p1(1); };

	double to_2PI(double yy){
		while(yy < 0){
			yy = yy < 0 ? yy + 2*M_PI : yy;
		}
		
		while(yy >= 2*M_PI){
			yy = yy >= 2*M_PI ? yy - 2*M_PI : yy;
		}

		return yy;
	};

	public:
	// Class Constructor
	Inspector(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Inspector();

	void inspect(Eigen::Matrix<double, 2, 1> pp);
	void getPath(std::vector<Eigen::Matrix<double, 3, 1>>& pps);
	void getTrajectory(std::vector<Eigen::Matrix<double, 3, 1>>& cp_, std::vector<double>& cpy_, double& tt_);
	void getInspectPoints(std::vector<Eigen::Matrix<double, 3, 1>>& pps);
};
}