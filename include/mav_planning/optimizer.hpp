#pragma once
#include <ros/ros.h>

#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <nlopt.hpp>

#include <mav_planning/environment.hpp>

class Optimizer{
	private:
	Environment* environment;

	int cost_function;
	int alg_1_ = 15; // NEWTON
	int alg_2_ = 11; // BFGS

	int spline_order, n_variables, max_iter_n;
	double min_dist, dt, max_opt_time;
	double v_max, a_max, bb, tt;
	double lambda1_, lambda2_, lambda3_,
		   lambda4_, lambda5_, lambda6_;

	bool endpoint = false;

	std::vector<Eigen::Matrix<double, 3, 1>> init_cp;
	std::vector<Eigen::Matrix<double, 3, 1>> opt_cp;
	std::vector<Eigen::Matrix<double, 3, 1>> wp;
	std::vector<Eigen::Matrix<double, 3, 1>> guide_pts;
	std::vector<double> tt_wp;
	std::vector<double> kk;
	Eigen::Matrix<double, 3, 1> end_pt;

	void optimize();
	static double lossFunction(const std::vector<double>& x, std::vector<double>& grad, void* data);
	void computeCost(const std::vector<double>& x, std::vector<double>& grad, double& cost);
	double smoothnessCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad);
	double distanceCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad);
	double feasibilityCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad);
	double endpointCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad);
	double guideCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad);
	double waypointsCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad);

	double deBoor(int n, int k, double time, std::vector<double> knot);
	void evaluateCurve(std::vector<Eigen::Matrix<double, 3, 1>> cp, double _tt, std::vector<double>& bbs, Eigen::Matrix<double, 3, 1>& rr);
	
	void reset();
	bool isLossQuadratic(){
		if(cost_function == GUIDE_PHASE){
			return true;
		} else if(cost_function == (SMOOTHNESS | WAYPOINTS)){
			return true;
		}

		return false;
	}

	public:
	// Class Constructor
	Optimizer(ros::NodeHandle& nh, Environment* _env);

	// Class Destructor
	~Optimizer();

	static const int SMOOTHNESS = (1 << 0);
  	static const int DISTANCE = (1 << 1);
  	static const int FEASIBILITY = (1 << 2);
  	static const int ENDPOINT = (1 << 3);
  	static const int GUIDE = (1 << 4);
  	static const int WAYPOINTS = (1 << 5);

  	static const int GUIDE_PHASE = SMOOTHNESS | GUIDE;
  	static const int NORMAL_PHASE = SMOOTHNESS | DISTANCE | FEASIBILITY;

	void optimizeBSpline(std::vector<Eigen::Matrix<double, 3, 1>>& cps, double tt, int loss);
	void setGuidePoints(std::vector<Eigen::Matrix<double, 3, 1>> gpt_);
	void setWayPoints(std::vector<Eigen::Matrix<double, 3, 1>> wp_, std::vector<double> tt_wp_);
	void setEndPoint(Eigen::Matrix<double, 3, 1> pp);
	void setCosts(Eigen::Matrix<double, 6, 1>& cc);
	void getCosts(Eigen::Matrix<double, 6, 1>& cc);
	void setVelocityBound(double vv_max);
	void setAccelerationBound(double aa_max);
};
