#include <mav_planning/optimizer.hpp>

// Constructor
Optimizer::Optimizer(ros::NodeHandle& nh, Environment* _env): environment(_env){
	nh.param("smooth_cost", lambda1_, 10.0);
	nh.param("distance_cost", lambda2_, 5.0);
	nh.param("feasibility_cost", lambda3_, 0.00001);
	nh.param("endpoint_cost", lambda4_, 0.01);
	nh.param("guide_cost", lambda5_, 100.0);
	nh.param("waypoint_cost", lambda6_, 5.0);
	nh.param("safe_margin", min_dist, 0.5);
	nh.param("max_acceleration", a_max, 0.5);
	nh.param("max_velocity", v_max, 1.0);
	nh.param("spline_order", spline_order, 7);
	nh.param("max_iterations", max_iter_n, 100);
	nh.param("max_opt_time", max_opt_time, 0.1);
	nh.param("variable_bound", bb, 5.0);
}

// Destructor
Optimizer::~Optimizer(){
  	;
}

void Optimizer::getCosts(Eigen::Matrix<double, 6, 1>& cc){
	cc(0) = lambda1_;
	cc(1) = lambda2_;
	cc(2) = lambda3_;
	cc(3) = lambda4_;
	cc(4) = lambda5_;
	cc(5) = lambda6_;
}

void Optimizer::setCosts(Eigen::Matrix<double, 6, 1>& cc){
	lambda1_ = cc(0);
	lambda2_ = cc(1);
	lambda3_ = cc(2);
	lambda4_ = cc(3);
	lambda5_ = cc(4);
	lambda6_ = cc(5);
}

void Optimizer::setVelocityBound(double vv_max){
	v_max = vv_max;
}

void Optimizer::setAccelerationBound(double aa_max){
	a_max = aa_max;
}

void Optimizer::setGuidePoints(std::vector<Eigen::Matrix<double, 3, 1>> gpt_){
	guide_pts.clear();
	guide_pts.shrink_to_fit();
	guide_pts = gpt_;
}

void Optimizer::setWayPoints(std::vector<Eigen::Matrix<double, 3, 1>> wp_, std::vector<double> tt_wp_){
	wp.clear();
	wp.shrink_to_fit();
	wp = wp_;

	tt_wp.clear();
	tt_wp.shrink_to_fit();
	tt_wp = tt_wp_;
}

void Optimizer::setEndPoint(Eigen::Matrix<double, 3, 1> pp){
	endpoint = true;
	end_pt = pp;
}

void Optimizer::optimizeBSpline(std::vector<Eigen::Matrix<double, 3, 1>>& cps, double _tt, int loss){
  	init_cp = cps;
	cost_function = loss;
	dt = _tt/(cps.size() - spline_order);
	tt = _tt;

	// Generate Equally Spaced Knots
	kk.resize(cps.size() + (spline_order+1));
	for(uint ii = 0; ii < cps.size() + (spline_order+1); ii++){
		if(ii <= spline_order)
			kk[ii] = 0;
		else if(ii > kk.size() - (spline_order+1))
			kk[ii] = kk[ii-1];
		else
			kk[ii] = kk[ii-1] + dt;
	}

	optimize();

	cps = opt_cp;
	reset();
}

void Optimizer::optimize(){
	if(cost_function & ENDPOINT){
		if(!endpoint){
			ROS_ERROR("[OPTIMIZER]: Endpoint not set");
			exit(-1);
		}

		n_variables = 3*(init_cp.size() - (spline_order+1)/2);
	} else{
		n_variables = 3*(init_cp.size() - (spline_order+1));
	}

	if(cost_function & GUIDE){
		if(guide_pts.size() == 0){
			ROS_ERROR("[OPTIMIZER]: Guidepoints not set");
			exit(-1);
		}
	}

	if(cost_function & WAYPOINTS){
		if(wp.size() == 0){
			ROS_ERROR("[OPTIMIZER]: Waypoints not set");
			exit(-1);
		}
	}

	std::vector<double> x(n_variables);
  	for(int ii = 0; ii < n_variables/3; ii++){
		x[ii*3 + 0] = init_cp[(spline_order+1)/2 + ii](0);
		x[ii*3 + 1] = init_cp[(spline_order+1)/2 + ii](1);
		x[ii*3 + 2] = init_cp[(spline_order+1)/2 + ii](2);
  	}

	int alg_ = isLossQuadratic() ? alg_1_:alg_2_;
  	nlopt::opt opt(nlopt::algorithm(alg_), n_variables);
  	opt.set_min_objective(Optimizer::lossFunction, this);
	opt.set_maxeval(max_iter_n);
  	opt.set_maxtime(max_opt_time);
  	opt.set_xtol_rel(1e-5);

	std::vector<double> lb(n_variables), ub(n_variables);
	for(int ii = 0; ii < n_variables; ii++){
		lb[ii] = x[ii] - bb;
		ub[ii] = x[ii] + bb;
	}

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	try{
		double fc;
		nlopt::result result = opt.optimize(x, fc);
		ROS_INFO("[OPTIMIZER]: Optimization Done");
	} catch(std::exception& e){
		ROS_ERROR("[OPTIMIZER]: NLOPT Failed, %s", e.what());
	}

	opt_cp = init_cp;
	for(int ii = 0; ii < n_variables/3; ii++){
		opt_cp[(spline_order+1)/2 + ii](0) = x[ii*3 + 0];
		opt_cp[(spline_order+1)/2 + ii](1) = x[ii*3 + 1];
		opt_cp[(spline_order+1)/2 + ii](2) = x[ii*3 + 2];
	}
}

double Optimizer::lossFunction(const std::vector<double>& x, std::vector<double>& grad, void* data){
	Optimizer* opt = reinterpret_cast<Optimizer*>(data);

	double cost;
	opt->computeCost(x, grad, cost);

	return cost;
}

void Optimizer::computeCost(const std::vector<double>& x, std::vector<double>& grad, double& cost){
	// Convert x in Points 3D Vector
	std::vector<Eigen::Matrix<double, 3, 1>> pps;
	pps.resize(init_cp.size());

	int ss = (spline_order+1)/2;
	int vv = int(n_variables/3);
	for(uint ii = 0; ii < ss; ii++){
		pps[ii] = init_cp[ii];
	}

	for(uint ii = 0; ii < vv; ii++){
		Eigen::Matrix<double, 3, 1> pp;
		pp(0) = x[ii*3 + 0];
		pp(1) = x[ii*3 + 1];
		pp(2) = x[ii*3 + 2];

		pps[ii + ss] = pp;
	}

	if(!(cost_function & ENDPOINT)){
		for(uint ii = 0; ii < ss; ii++){
			pps[ii + ss + vv] = init_cp[ii + ss + vv];
		}
	}

	// Initialize Cost and Gradient to Zero
	cost = 0.0;
	grad.resize(x.size());
	std::fill(grad.begin(), grad.end(), 0.0);

	if(cost_function & SMOOTHNESS){
		std::vector<Eigen::Matrix<double, 3, 1>> grad_;
		grad_.resize(pps.size());

		cost += lambda1_*smoothnessCost(pps, grad_);
		for(int ii = 0; ii < vv; ii++){
			grad[ii*3 + 0] += lambda1_*grad_[ii+ss](0);
			grad[ii*3 + 1] += lambda1_*grad_[ii+ss](1);
			grad[ii*3 + 2] += lambda1_*grad_[ii+ss](2);
		}
	}

	if(cost_function & DISTANCE){
		std::vector<Eigen::Matrix<double, 3, 1>> grad_;
		grad_.resize(pps.size());

		cost += lambda2_*distanceCost(pps, grad_);
		for(int ii = 0; ii < vv; ii++){
			grad[ii*3 + 0] += lambda2_*grad_[ii+ss](0);
			grad[ii*3 + 1] += lambda2_*grad_[ii+ss](1);
			grad[ii*3 + 2] += lambda2_*grad_[ii+ss](2);
		}
	}

	if(cost_function & FEASIBILITY){
		std::vector<Eigen::Matrix<double, 3, 1>> grad_;
		grad_.resize(pps.size());

		cost += lambda3_*feasibilityCost(pps, grad_);
		for(int ii = 0; ii < vv; ii++){
			grad[ii*3 + 0] += lambda3_*grad_[ii+ss](0);
			grad[ii*3 + 1] += lambda3_*grad_[ii+ss](1);
			grad[ii*3 + 2] += lambda3_*grad_[ii+ss](2);
		}
	}

	if(cost_function & ENDPOINT){
		std::vector<Eigen::Matrix<double, 3, 1>> grad_;
		grad_.resize(pps.size());

		cost += lambda4_*endpointCost(pps, grad_);
		for(int ii = 0; ii < vv; ii++){
			grad[ii*3 + 0] += lambda4_*grad_[ii+ss](0);
			grad[ii*3 + 1] += lambda4_*grad_[ii+ss](1);
			grad[ii*3 + 2] += lambda4_*grad_[ii+ss](2);
		}
	}

	if(cost_function & GUIDE){
		std::vector<Eigen::Matrix<double, 3, 1>> grad_;
		grad_.resize(pps.size());

		cost += lambda5_*guideCost(pps, grad_);
		for(int ii = 0; ii < vv; ii++){
			grad[ii*3 + 0] += lambda5_*grad_[ii+ss](0);
			grad[ii*3 + 1] += lambda5_*grad_[ii+ss](1);
			grad[ii*3 + 2] += lambda5_*grad_[ii+ss](2);
		}
	}

	if(cost_function & WAYPOINTS){
		std::vector<Eigen::Matrix<double, 3, 1>> grad_;
		grad_.resize(pps.size());

		cost += lambda6_*waypointsCost(pps, grad_);
		for(int ii = 0; ii < vv; ii++){
			grad[ii*3 + 0] += lambda6_*grad_[ii+ss](0);
			grad[ii*3 + 1] += lambda6_*grad_[ii+ss](1);
			grad[ii*3 + 2] += lambda6_*grad_[ii+ss](2);
		}
	}
}

double Optimizer::smoothnessCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad){
	double cost = 0.0;
	std::fill(grad.begin(), grad.end(), Eigen::MatrixXd::Zero(3,1));

	Eigen::Matrix<double, 3, 1> jj;
	Eigen::Matrix<double, 3, 1> ww;
	for(uint ii = 0; ii < cp.size()-3; ii++){
		jj = cp[ii+3] - 3*cp[ii+2] + 3*cp[ii+1] - cp[ii];
		cost += jj.squaredNorm();

		ww = 2.0*jj;
		grad[ii + 0] += -ww;
		grad[ii + 1] += 3.0*ww;
		grad[ii + 2] += -3.0*ww;
		grad[ii + 3] += ww;
	}

	return cost;
}

double Optimizer::distanceCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad){
	double cost = 0.0;
	std::fill(grad.begin(), grad.end(), Eigen::MatrixXd::Zero(3,1));

	uint ss = (spline_order+1)/2;
	uint se = (cost_function & ENDPOINT) ? cp.size():cp.size()-ss;
	Eigen::Matrix<double, 3, 1> grad_;

	for(uint ii = ss; ii < se; ii++){
		double dd = environment->evaluateSDF_withGradient(cp[ii](0), cp[ii](1), cp[ii](2), grad_);

		if(grad_.norm() > 1e-4){
			grad_.normalize();
		}

		if(dd < min_dist){
			cost += pow(dd - min_dist, 2);
			grad[ii] += 2.0*(dd - min_dist)*grad_;
		}
	}

	return cost;
}

double Optimizer::feasibilityCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad){
	double cost = 0.0;
	std::fill(grad.begin(), grad.end(), Eigen::MatrixXd::Zero(3,1));

	double v_max_sq = v_max*v_max;
	double a_max_sq = a_max*a_max;
	double dt_inv_sq = 1/(dt*dt);
	double dt_inv_sqsq = dt_inv_sq*dt_inv_sq;

	// Velocity
	Eigen::Matrix<double, 3, 1> v;
	for(int ii = 0; ii < cp.size()-1; ii++) {
		v = cp[ii+1] - cp[ii];

		for(int jj = 0; jj < 3; jj++) {
			double dv = v(jj)*v(jj)*dt_inv_sq - v_max_sq;
			if(dv > 0.0){
				cost += pow(dv, 2);

				double ww = 4.0*dv*dt_inv_sq;
				grad[ii + 0](jj) += -ww*v(jj);
				grad[ii + 1](jj) += ww*v(jj);
			}
		}
	}

	// Acceleration
	Eigen::Matrix<double, 3, 1> a;
	for(int ii = 0; ii < cp.size()-2; ii++){
		a = cp[ii+2] - 2*cp[ii+1] + cp[ii];

		for(int jj = 0; jj < 3; jj++){
			double da = a(jj)*a(jj)*dt_inv_sqsq - a_max_sq;
			if(da > 0.0){
				cost += pow(da, 2);

				double ww = 4.0*da*dt_inv_sqsq;
				grad[ii + 0](jj) += ww*a(jj);
				grad[ii + 1](jj) += -2.0*ww*a(jj);
				grad[ii + 2](jj) += ww*a(jj);
			}
		}
	}

	return cost;
}

double Optimizer::endpointCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad){
	double cost = 0.0;
	std::fill(grad.begin(), grad.end(), Eigen::MatrixXd::Zero(3,1));

	// zero cost and gradient in hard constraints
	Eigen::Matrix<double, 3, 1> q3, q2, q1, dq;
	q3 = cp[cp.size() - 3];
	q2 = cp[cp.size() - 2];
	q1 = cp[cp.size() - 1];

	dq = ((1/6.0)*(q3 + 4*q2 + q1)) - end_pt;
	cost += dq.squaredNorm();

	grad[cp.size() - 3] += 2.0*dq*(1/6.0);
	grad[cp.size() - 2] += 2.0*dq*(4/6.0);
	grad[cp.size() - 1] += 2.0*dq*(1/6.0);

	return cost;
}

double Optimizer::guideCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad){
	double cost = 0.0;
	std::fill(grad.begin(), grad.end(), Eigen::MatrixXd::Zero(3,1));

	uint ss = (spline_order+1)/2;
	uint se = cp.size()-ss;

	Eigen::Matrix<double, 3, 1> gpt;
	for(int ii = ss; ii < se; ii++){
		gpt = guide_pts[ii-ss];

		cost += (cp[ii] - gpt).squaredNorm();
		grad[ii] += 2.0*(cp[ii] - gpt);
	}

	return cost;
}

double Optimizer::waypointsCost(std::vector<Eigen::Matrix<double, 3, 1>> cp, std::vector<Eigen::Matrix<double, 3, 1>>& grad){
	double cost = 0.0;
	std::fill(grad.begin(), grad.end(), Eigen::MatrixXd::Zero(3,1));

	double tt_wpi;
	std::vector<double> bbs;
	Eigen::Matrix<double, 3, 1> wpi;
	Eigen::Matrix<double, 3, 1> pp;
	for(int ii = 0; ii < wp.size(); ii++){
		wpi = wp[ii];
		tt_wpi = tt_wp[ii];

		evaluateCurve(cp, tt_wpi, bbs, pp);
		Eigen::Matrix<double, 3, 1> dq = pp - wpi;
		cost += dq.squaredNorm();

		for(uint ii = 0; ii < cp.size(); ii++){
			grad[ii] += 2*dq*bbs[ii];
		}
	}

	return cost;
}

void Optimizer::evaluateCurve(std::vector<Eigen::Matrix<double, 3, 1>> cp, double _tt, std::vector<double>& bbs, Eigen::Matrix<double, 3, 1>& rr){
	int ss = cp.size();
	double tr = 0.0;
	if(_tt >= tt){
		tr = tt - 1e-5;
	} else{
		tr = _tt;
	}

	bbs.resize(ss);
	rr = Eigen::MatrixXd::Zero(3, 1);
	for(uint ii = 0; ii < ss; ii++){
		bbs[ii] = deBoor(spline_order, ii, tr, kk);
		rr += cp[ii]*bbs[ii];
	}
}

double Optimizer::deBoor(int n, int k, double time, std::vector<double> knot){
	double c1, c2;

	if(n == 0){
		if((time >= knot[k]) && (time < knot[k+1]))
			return 1;
		else
			return 0;
	} else{
		if(knot[k+n] - knot[k] != 0)
			c1 = (time - knot[k])/(knot[k+n] - knot[k]);
		else
			c1 = 0;

		if(knot[k+n+1] - knot[k+1] != 0)
			c2 = (knot[k+n+1] - time)/(knot[k+n+1] - knot[k+1]);
		else
			c2 = 0;
		
		return c1*deBoor(n-1, k, time, knot) + c2*deBoor(n-1, k+1, time, knot);
	}
}


void Optimizer::reset(){
	init_cp.clear();
	init_cp.shrink_to_fit();

	opt_cp.clear();
	opt_cp.shrink_to_fit();

	wp.clear();
	wp.shrink_to_fit();

	guide_pts.clear();
	guide_pts.shrink_to_fit();

	tt_wp.clear();
	tt_wp.shrink_to_fit();

	endpoint = false;
}


