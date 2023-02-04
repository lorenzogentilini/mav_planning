#pragma once
#include <ros/ros.h>

#include <random>
#include <vector>
#include <queue>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <mav_planning/containers.hpp>
#include <mav_planning/optimizer.hpp>
#include <mav_planning/environment.hpp>

namespace _Planner_{
#define OPEN_EMPTY   -1
#define MEMORY_OUT   -2
#define NO_SHOT_TRAJ -3
#define GOAL_REACHED  1
#define SEARCHING     0

// Template (Hash Function) to Adapt unordered_map to Eigen::Matrix
template <class T>
struct matrixHash : std::unary_function<T, size_t>{
	std::size_t operator()(T const& matrix) const{
		size_t seed = 0;
		for(size_t i = 0; i < (size_t)matrix.size(); ++i) {
			auto elem = *(matrix.data() + i);
			seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		return seed;
	}
};

class Node{
	public:
	// Node Attributes
	Eigen::Matrix<int, 3, 1> index;
	Eigen::Matrix<double, 6, 1> state;
	Eigen::Matrix<double, 3, 1> input;
	double g_value, f_value, time;
	Node* nodeParent;

	// Class Constructor
	Node(){
		nodeParent = NULL;
		g_value = INFINITY;
		f_value = INFINITY;
	}

	// Class Destructor
	~Node(){;}
};

class NodeComparator{
	public:
	bool operator()(Node* node1, Node* node2) {
		return node1->f_value > node2->f_value;
	}
};

class Planner{
	private:
	double lambda, v_max, a_max, a_res;
	double tt_min, tt_max, tt_res, dt;
	double rho, safe_margin, m_res;
	double min_tol, delta_sampling;
	double m_max_x, m_max_y, m_max_z;
	double m_min_x, m_min_y, m_min_z;
	int max_nodeNum;

	int usedNode = 0;
	int global_flag = SEARCHING;
	double optimal_trajTime;

	std::vector<Eigen::Matrix<double, 3, 1>> input;
	std::vector<double> timeInt;
	Eigen::Matrix<double, 6, 1> goal;
	Eigen::Matrix<double, 3, 4> p_coeff;

	std::priority_queue<Node*, std::vector<Node*>, NodeComparator> openSet;
	std::unordered_map<Eigen::Matrix<int, 3, 1>, Node*, matrixHash<Eigen::Matrix<int, 3, 1>>> openMap;
	std::unordered_map<Eigen::Matrix<int, 3, 1>, Node*, matrixHash<Eigen::Matrix<int, 3, 1>>> closedSet;
	std::vector<Node*> nodePath;
	std::vector<Node*> nodePool;
	
	Environment* environment;
	Optimizer* optimizer;
	BSpline<Eigen::Matrix<double, 3, 1>>* trajectory;

	void expandNode(Node* node);
	double compute_hValue(Eigen::Matrix<double, 6, 1> xc, Eigen::Matrix<double, 6, 1> xg, double& optimal_time);
	void statePropagation(Eigen::Matrix<double, 6, 1>& xi, Eigen::Matrix<double, 6, 1>& xf, Eigen::Matrix<double, 3, 1> u, double tau);
	bool checkCollision(Eigen::Matrix<double, 3, 1> p);
	void retrieveNodePath(Node* node);
	Eigen::Matrix<int, 3, 1> indexFromPose(Eigen::Matrix<double, 3, 1> pt);
	std::vector<double> computeRoot(double a, double b, double c, double d, double e);
	bool oneShotTraj(Eigen::Matrix<double, 6, 1> xc, Eigen::Matrix<double, 6, 1> xg, double optimal_time);

	public:
	// Class Constructor
	Planner(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Planner();

	int search(Eigen::Matrix<double, 6, 1> start, Eigen::Matrix<double, 6, 1> goal);
	double getTraj(std::vector<Eigen::Matrix<double, 3, 1>>& sampledTraj_, double stepSize);
	void getTrajectory(std::vector<Eigen::Matrix<double, 3, 1>>& cp_, double& tt_);
	void reset();
};
}