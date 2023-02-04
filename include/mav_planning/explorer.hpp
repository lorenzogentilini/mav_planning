#pragma once
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <vector>
#include <queue>
#include <random>
#include <unordered_map>

#include <mav_planning/containers.hpp>
#include <mav_planning/optimizer.hpp>
#include <mav_planning/environment.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace _Explorer_{
class CostMap{
	public:
	// Class Constructor
	CostMap();

	// Class Destructor
	~CostMap();

	int m_gridSize_x, m_gridSize_y;
	int m_gridSize, m_gridStep_y;
	double m_max_x, m_max_y, m_max_z;
	double m_min_x, m_min_y, m_min_z;
	double cc_inc, cc_dec, m_res, rr;
	
	std::vector<double> m_grid;

	static const int INCREASE = (1 << 0);
	static const int DECREASE = (1 << 1);

	double evaluateCost(double x, double y);
	double evaluateCostRadius(double x, double y);
	void updateCost(double x, double y, int action);
	void updateCostRadius(double x, double y, int action);
	void updateCostFromPoints(std::vector<Eigen::Matrix<double, 2, 1>> ps, int action);
	void updateAllCostMap(int action);
	double getValueIdx(int idx);
	bool checkSimilarity(CostMap costmap);
	int getGridSize_x();
	int getGridSize_y();

	inline int point2grid(double x, double y);
	inline bool isIntoMap(double x, double y);

	CostMap copyTo();
	void swap(CostMap costmap);
};

class CostMapContainer: public CostMap{
	private:
	double flight_altitude;
	double m_cf;

	bool debug_flag;

	Environment* environment;

	ros::Publisher visPublisher;
	ros::Timer pubTimer;

	void computeGrid();
	void publishMap();
	Eigen::Matrix<double, 4, 1> heightColor(double h);
	void timerPublish(const ros::TimerEvent& e);

	public:
	// Class Constructor
	CostMapContainer(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	virtual ~CostMapContainer();

	void reset();
};

class Node{
	public:
	// Node Attributes
	double ig = 0.0;
	double cc = 0.0;

	Node* nodeParent = this;
	Eigen::Matrix<double, 2, 1> pose;
	CostMap costmap; // Copy of CostMap

	// Class Constructor
	Node(){;}

	// Class Destructor
	~Node(){;}

	double getTotalUtility(){
		if(cc <= 0.0){
			return 0.0;
		}

		return getBranchGain()/getBranchCost();
	}

	double getBranchCost(){
		if(nodeParent == this){
			return cc;
		} else{
			return nodeParent->getBranchCost() + cc;
		}
	}

	double getBranchGain(){
		if(nodeParent == this){
			return ig;
		} else{
			return nodeParent->getBranchGain() + ig;
		}
	}
};

class NodeComparator{
	public:
	bool operator()(Node* node1, Node* node2) {
		return node1->getTotalUtility() < node2->getTotalUtility();
	}
};

class Explorer{
	private:
	double max_rr, min_rr, safe_margin;
	double m_max_x, m_max_y;
	double m_min_x, m_min_y;
	double vv_max, s_res;

	double flight_altitude;
	bool debug_flag;
	int max_n_nodes, n_childrens;

	Environment* environment;
	CostMapContainer* costmap;
	Optimizer* optimizer;
	Node* bestNode;
	CostMap old_costmap;

	std::vector<Node*> nodeTree;
	std::vector<Eigen::Matrix<double, 3, 1>> way_points;
	std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open_set;

	bool insideBounds(Eigen::Matrix<double, 2, 1> pp);
	void samplePoint(Eigen::Matrix<double, 2, 1>& np, Eigen::Matrix<double, 2, 1> pp);
	void initializeTree(Eigen::Matrix<double, 2, 1> pose);
	void growTree(Eigen::Matrix<double, 2, 1> pose);
	bool isLineCollision(Eigen::Matrix<double, 2, 1> pi, Eigen::Matrix<double, 2, 1> pf);
	void getSamples(Eigen::Matrix<double, 2, 1> pi, Eigen::Matrix<double, 2, 1> pf, std::vector<Eigen::Matrix<double, 2, 1>>& pps);
	void discretizePath(std::vector<Eigen::Matrix<double, 3, 1>>& wp);
	double pathLenght();
	
	public:
	// Class Constructor
	Explorer(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Explorer();

	void explore(Eigen::Matrix<double, 2, 1> pose, std::vector<Eigen::Matrix<double, 3, 1>>& wp);
	void fitWithBSpline(std::vector<Eigen::Matrix<double, 3, 1>>& cp, double& tt);
	void stepBack();
	void reset();
};
}