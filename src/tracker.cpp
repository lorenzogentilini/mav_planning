#include <mav_planning/tracker.hpp>

using namespace _Tracker_;

// Constructor
Tracker::Tracker(ros::NodeHandle& nh, Environment* _env): environment(_env),
	poseSubscriber(nh.subscribe("/cam_to_marker", 1, &Tracker::poseCallback, this)){

	nh.param("robot_id", robot_id, 30);
	nh.param("max_dt", max_dt, 3.0);
	nh.param("robot_radius", rb_rr, 0.3);
	nh.param("map_resolution", mm_res, 0.2);
	nh.param("safe_margin", safe_margin, 0.55);
	nh.param("points_number", n_pp, 5);
	nh.param("look_ahead_time", tt_max, 0.2);
	nh.param("takeoff_altitude", takeoff_altitude, 1.7);

	nh.param("kp", KP, 0.5);
	nh.param("ki", KI, 0.3);

	nh.param("kp_yaw", KP_yy, 0.5);
	nh.param("ki_yaw", KI_yy, 0.3);

	nh.param("publish_debug_tra", debug_flag, true);

	kalman = new EKF(nh);
	optimizer = new Optimizer(nh, environment);

	spline = new BSpline<Eigen::Matrix<double, 3, 1>>(7);
	spline_yy = new BSpline<double>(3);
}

// Destructor
Tracker::~Tracker(){
	delete kalman;
	delete optimizer;

	delete spline;
	delete spline_yy;
}

void Tracker::reset(){
	spline->clearAll();
	spline_yy->clearAll();
}

void Tracker::getPredictedRoverPose(std::vector<Eigen::Matrix<double, 3, 1>>& _rp, std::vector<double>& _ry){
	std::vector<Eigen::Matrix<double, 3, 1>> rps;
	std::vector<double> rys;
	for(uint ii = 0; ii <  rp.size(); ii++){
		Eigen::Matrix<double, 3, 1> pp;
		pp(0) = rp[ii](0);
		pp(1) = rp[ii](1);
		pp(2) = 0.0;

		rps.push_back(pp);
		rys.push_back(rp_yy[ii]);
	}

	_rp = rps;
	_ry = rys;
}

bool Tracker::track(double ti, std::vector<Eigen::Matrix<double, 3, 1>>& _cp, std::vector<double>& _cpy, double& _tt){
	if(!poseValid){
		ROS_WARN_COND(debug_flag, "[TRACKER]: Impossible to Predict Rover Positions");
		return false;
	}

	if(!newPose){ // Is It Necessary?
		return true;
	}

	pps.clear();
	pps.shrink_to_fit();

	pps_yy.clear();
	pps_yy.shrink_to_fit();

	rp.clear();
	rp.shrink_to_fit();

	rp_yy.clear();
	rp_yy.shrink_to_fit();

	wp.clear();
	wp.shrink_to_fit();

	wp_yy.clear();
	wp_yy.shrink_to_fit();

	wp_tt.clear();
	wp_tt.shrink_to_fit();

	Eigen::Matrix<double, 3, 1> init_vv;
	Eigen::Matrix<double, 3, 1> last_vv;
	predictRoverPosition(init_vv, last_vv);

	// Compute PPS
	if(spline->getControl().size() != 0){
		double dt = wp_tt[0];
		for(uint ii = 0; ii < rp.size(); ii++){
			Eigen::Matrix<double, 3, 1> pp = spline->evaluateCurve(ti + dt*ii, 0);
			pps.push_back(pp);
		}

	} else{
		tf2::Vector3 mapToBody_translation = mapToBody_transformation.getOrigin();

		Eigen::Matrix<double, 3, 1> pp;
		pp(0) = mapToBody_translation.x();
		pp(1) = mapToBody_translation.y();
		pp(2) = mapToBody_translation.z();

		for(uint ii = 0; ii < rp.size(); ii++){
			pps.push_back(pp);
		}
	}

	// Compute PPS_YY
	if(spline_yy->getControl().size() != 0){
		double dt = wp_tt[0];
		for(uint ii = 0; ii < rp_yy.size(); ii++){
			double yy = spline_yy->evaluateCurve(ti + dt*ii, 0);
			pps_yy.push_back(yy);
		}

	} else{
		tf2::Quaternion mapToBody_quaternion = mapToBody_transformation.getRotation();
		tf2::Matrix3x3 m(mapToBody_quaternion);

		double rr, pp, yy;
		m.getRPY(rr, pp, yy);

		for(uint ii = 0; ii < rp_yy.size(); ii++){
			pps_yy.push_back(yy);
		}
	}

	// Generate Waypoints
	getWaypoints();

	if(wp.size() == 0 || wp_yy.size() == 0){
		ROS_WARN_COND(debug_flag, "[TRACKER]: No Waypoints Found");
		return false;
	}

	// Handle XYZ Trajectory
	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> bb;
	std::vector<Eigen::Matrix<double, 3, 1>> vv;
	std::vector<Eigen::Matrix<double, 3, 1>> aa;
	std::vector<Eigen::Matrix<double, 3, 1>> jj;

	Eigen::Matrix<double, 3, 1> zero = Eigen::MatrixXd::Zero(3, 1);
	Eigen::Matrix<double, 3, 1> pi;
	Eigen::Matrix<double, 3, 1> vi;
	Eigen::Matrix<double, 3, 1> vf;

	vi(0) = init_vv(0);
	vi(1) = init_vv(1);
	vi(2) = 0.0;

	vf(0) = last_vv(0);
	vf(1) = last_vv(1);
	vf(2) = 0.0;

	if(spline->getControl().size() != 0){
		pi = spline->evaluateCurve(ti, 0);
		wp.insert(wp.begin(), pi);
		wp_tt.insert(wp_tt.begin(), 0.0);

		vv.push_back(spline->evaluateCurve(ti, 1));
		vv.push_back(vf);
		aa.push_back(spline->evaluateCurve(ti, 2));
		aa.push_back(zero);
		jj.push_back(spline->evaluateCurve(ti, 3));
		jj.push_back(zero);

	} else{
		tf2::Vector3 mapToBody_translation = mapToBody_transformation.getOrigin();
		pi(0) = mapToBody_translation.x();
		pi(1) = mapToBody_translation.y();
		pi(2) = mapToBody_translation.z();

		wp.insert(wp.begin(), pi);
		wp_tt.insert(wp_tt.begin(), 0.0);

		vv.push_back(vi);
		vv.push_back(vf);
		aa.push_back(zero);
		aa.push_back(zero); 
		jj.push_back(zero);
		jj.push_back(zero);
	}

	bb.push_back(vv);
	bb.push_back(aa);
	bb.push_back(jj);

	spline->setWaypoints(wp, bb, tt_max);

	std::vector<Eigen::Matrix<double, 3, 1>> cp = spline->getControl();
	optimizer->setWayPoints(wp, wp_tt);
	optimizer->optimizeBSpline(cp, tt_max, Optimizer::NORMAL_PHASE | Optimizer::WAYPOINTS);

	spline->setControl(cp, tt_max);

	_tt = tt_max;
	_cp = cp;

	// Handle Yaw Trajectory
	std::vector<std::vector<double>> bb_yy;
	std::vector<double> vv_yy;

	if(spline_yy->getControl().size() != 0){
		double yi = spline_yy->evaluateCurve(ti, 0);
		wp_yy.insert(wp_yy.begin(), to_2PI(yi));

		vv_yy.push_back(0.0);
		vv_yy.push_back(0.0);

	} else{
		tf2::Quaternion mapToBody_quaternion = mapToBody_transformation.getRotation();
		tf2::Matrix3x3 m(mapToBody_quaternion);

		double rr, pp, yy;
		m.getRPY(rr, pp, yy);

		wp_yy.insert(wp_yy.begin(), to_2PI(yy));

		vv_yy.push_back(0.0);
		vv_yy.push_back(0.0);
	}

	bb_yy.push_back(vv_yy);

	spline_yy->setWaypoints(wp_yy, bb_yy, tt_max);

	std::vector<double> cp_yy = spline_yy->getControl();
	_cpy = cp_yy;

	newPose = false;
	return true;
}

void Tracker::predictRoverPosition(Eigen::Matrix<double, 3, 1>& init_vv, Eigen::Matrix<double, 3, 1>& last_vv){
	// Update Current Estimation
	kalman->update();

	if(debug_flag){
		Eigen::Matrix<double, 5, 1> xx;
		kalman->getState(xx);

		ROS_INFO("[TRACKER]: Predicted Pose %f %f %f", xx(0), xx(1), xx(3));
		ROS_INFO("[TRACKER]: Predicted Velocity %f %f", xx(2), xx(4));
	}

	double ts = tt_max/(double)n_pp;
	std::vector<Eigen::Matrix<double, 5, 1>> xxs;
	kalman->predict(xxs, ts, tt_max);

	if(xxs.size() == 0){
		return;
	}

	Eigen::Matrix<double, 2, 1> rpi;
	rpi(0) = xxs[0](0);
	rpi(1) = xxs[0](1);

	double rpi_yy = xxs[0](3);

	for(uint ii = 0; ii < xxs.size(); ii++){
		if(environment->evaluateSDF(xxs[ii](0), xxs[ii](1), rb_rr+(2*mm_res)) < rb_rr){
			int miss = n_pp - rp.size();
			if(miss != 0){
				for(uint jj = 0; jj < miss; jj++){
					rp.push_back(rpi);
					rp_yy.push_back(rpi_yy);
					wp_tt.push_back(ts*ii);
				}
			}

			break;
		}

		rpi(0) = xxs[ii](0);
		rpi(1) = xxs[ii](1);

		rpi_yy = xxs[ii](3);

		rp.push_back(rpi);
		rp_yy.push_back(rpi_yy);
		wp_tt.push_back(ts*(ii+1));
	}

	init_vv(0) = xxs.front()(2)*cos(xxs.front()(3));
	init_vv(1) = xxs.front()(2)*sin(xxs.front()(3));
	init_vv(2) = xxs.front()(4);

	last_vv(0) = xxs.back()(2)*cos(xxs.back()(3));
	last_vv(1) = xxs.back()(2)*sin(xxs.back()(3));
	last_vv(2) = xxs.back()(4);
}

void Tracker::getWaypoints(){
	if(rp.size() == 0 || rp_yy.size() == 0){
		return;
	}
	
	wp.resize(rp.size());
	Eigen::Matrix<double, 3, 1> pp;
	Eigen::Matrix<double, 3, 1> current_err;
	Eigen::Matrix<double, 3, 1> grad;

	for(uint ii = 0; ii < rp.size(); ii++){
		pp(0) = rp[ii](0);
		pp(1) = rp[ii](1);
		pp(2) = takeoff_altitude;

		current_err = pp-pps[ii];
		if(ii == 0){
			track_error += current_err*wp_tt[0];
		}

		pp = pps[ii] + current_err*KP + track_error*KI;

		double dd = environment->evaluateSDF_withGradient(pp(0), pp(1), pp(2), grad);
		if(dd < safe_margin){
			pp += grad*abs(dd-safe_margin);
		}
	
		wp[ii] = pp;
	}

	wp_yy.resize(rp_yy.size());
	double current_err_yy;

	for(uint ii = 0; ii < rp_yy.size(); ii++){
		rp_yy[ii] = to_2PI(rp_yy[ii]);
		pps_yy[ii] = to_2PI(pps_yy[ii]);

		double dy = rp_yy[ii] - pps_yy[ii];
		double dy_2PI = abs(dy) - 2*M_PI;

		if(dy < 0){
			dy_2PI = -dy_2PI;
		}

		current_err_yy = abs(dy) < abs(dy_2PI) ? dy : dy_2PI;
		if(ii == 0){
			track_error_yy += current_err_yy*wp_tt[0];
		}

		wp_yy[ii] = pps_yy[ii] + current_err_yy*KP_yy + track_error_yy*KI_yy;
	}
}

void Tracker::poseCallback(const nav_msgs::Odometry::ConstPtr& pose){
	ros::Time tt = ros::Time::now();

	tf2::Transform odomToBody_transformation;
	if(!environment->getLastTransform(mapToOdom_transformation) || !environment->getLastOdomTransform(odomToBody_transformation)){
		return;
	}

	mapToBody_transformation = mapToOdom_transformation*odomToBody_transformation;

	int id = stoi(pose->child_frame_id);
	if(id != robot_id){
		return;
	}

	if((tt-last_tt).toSec() > max_dt){
		poseValid = false;
	}

	// Convert CamToRobot To Global Robot Pose
	tf2::Vector3 bodyToRobot_translation(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z);
	tf2::Quaternion bodyToRobot_quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
	tf2::Transform bodyToRobot_transformation(bodyToRobot_quaternion, bodyToRobot_translation);
	tf2::Transform mapToRobot_transformation = mapToBody_transformation*bodyToRobot_transformation;

	tf2::Vector3 mapToRobot_translation = mapToRobot_transformation.getOrigin();
	tf2::Quaternion mapToRobot_quaternion = mapToRobot_transformation.getRotation();
	tf2::Matrix3x3 m(mapToRobot_quaternion);

	double r, p, y;
	m.getRPY(r,p,y);

	// Correct Angle Mismatch (I Don't Know Why, But it Works)
	y += M_PI/2;

	ROS_INFO_COND(debug_flag, "[TRACKER]: Measure: %f %f %f", mapToRobot_translation.x(), mapToRobot_translation.y(), y);

	if(!poseValid){
		Eigen::Matrix<double, 5, 1> xx = Eigen::MatrixXd::Zero(5, 1);

		xx(0) = mapToRobot_translation.x();
		xx(1) = mapToRobot_translation.y();
		xx(3) = y;

		kalman->reset(xx);

	} else{
		Eigen::Matrix<double, 3, 1> yy;
		Eigen::Matrix<double, 3, 3> rr = Eigen::MatrixXd::Identity(3, 3);

		yy(0) = mapToRobot_translation.x();
		yy(1) = mapToRobot_translation.y();
		yy(2) = y;

		rr(0,0) = 0.2;//pose->pose.covariance[0];
		rr(1,1) = 0.2;//pose->pose.covariance[0];
		rr(2,2) = 0.2;//pose->pose.covariance[0];

		kalman->correct(yy, rr);
	}

	poseValid = true;
	newPose = true;
	last_tt = ros::Time::now();
}

void Tracker::updateEKF_forDebug(Eigen::Matrix<double, 3, 1> yy){
	if(!poseValid){
		Eigen::Matrix<double, 5, 1> xx = Eigen::MatrixXd::Zero(5, 1);

		xx(0) = yy(0);
		xx(1) = yy(1);
		xx(3) = yy(2);

		kalman->reset(xx);

	} else{
		Eigen::Matrix<double, 3, 3> rr = Eigen::MatrixXd::Identity(3, 3)*0.1;
		kalman->correct(yy, rr);
	}

	poseValid = true;

	tf2::Vector3 mapToBody_translation(yy(0), yy(1), takeoff_altitude);
	mapToBody_transformation.setOrigin(mapToBody_translation);
}

// Constructor
EKF::EKF(ros::NodeHandle& nh){
	nh.param("pi_diag_0", pd(0), 1.0e-6);
	nh.param("pi_diag_1", pd(1), 1.0e-6);
	nh.param("pi_diag_2", pd(2), 1.0e-6);
	nh.param("pi_diag_3", pd(3), 1.0e-6);
	nh.param("pi_diag_4", pd(4), 1.0e-6);

	nh.param("qq_diag_0", qd(0), 0.01);
	nh.param("qq_diag_1", qd(1), 0.01);
	nh.param("qq_diag_2", qd(2), 0.05);
	nh.param("qq_diag_3", qd(3), 0.02);
	nh.param("qq_diag_4", qd(4), 0.05);

	nh.param("const_linear_deceleration", dv, 0.1);
	nh.param("const_angular_deceleration", dw, 0.2);

	id = Eigen::MatrixXd::Identity(5, 5);
	pi = Eigen::MatrixXd::Zero(5, 5);
	qq = Eigen::MatrixXd::Zero(5, 5);
	for(uint ii = 0; ii < 5; ii++){
		for(uint jj = 0; jj < 5; jj++){
			if(ii == jj){
				pi(ii,jj) = pd(ii);
				qq(ii,jj) = qd(ii);
			}
		}
	}

	hh << 1, 0, 0, 0, 0,
		  0, 1, 0, 0, 0,
		  0, 0, 0, 1, 0;
}

// Destructor
EKF::~EKF(){
	;
}

void EKF::reset(Eigen::Matrix<double, 5, 1> xx){
	xk = xx;
	pk = pi;

	previousIter = ros::Time::now();
	ready = true;
}

void EKF::getSystemDynamics(Eigen::Matrix<double, 5, 1>& xd, Eigen::Matrix<double, 5, 1> xx){
	xd(0) = cos(xx(3))*xx(2); 
	xd(1) = sin(xx(3))*xx(2); 
	xd(2) = -dv*xx(2); // Constant Deceleration Model
	xd(3) = xx(4);
	xd(4) = -dw*xx(4); // Constant Deceleration Model
}

void EKF::getJacobian(Eigen::Matrix<double, 5, 5>& ff, Eigen::Matrix<double, 5, 1> xx){
	ff << 0, 0, cos(xx(3)), -xx(2)*sin(xx(3)), 0,
		  0, 0, sin(xx(3)),  xx(2)*cos(xx(3)), 0,
		  0, 0, -dv, 		 0, 			   0,
		  0, 0, 0, 			 0, 			   1,
		  0, 0, 0, 			 0, 			   -dw;
}

void EKF::predict(std::vector<Eigen::Matrix<double, 5, 1>>& xx, double dt, double tt){
	// Propagate Continuous-Time Dynamics
	Eigen::Matrix<double, 5, 1> xd;
	Eigen::Matrix<double, 5, 1> xi;
	
	xi = xk;
	for(double ti = dt; ti <= tt+dt; ti += dt){
		// Propagation Step
		getSystemDynamics(xd, xi);
		xi = xi + xd*tt;

		xx.push_back(xi);
	}
}

void EKF::update(){
	if(!ready){
		ROS_WARN("[EKF]: Initial Condition Required, Please Perform Reset");
		return;
	}

	actualIter = ros::Time::now();

	Eigen::Matrix<double, 5, 1> xd;
	Eigen::Matrix<double, 5, 5> fk;

	// Propagate Continuous-Time Dynamics
	getSystemDynamics(xd, xk);

	// Jacobian
	getJacobian(fk, xk);
	fk = fk*(actualIter-previousIter).toSec() + Eigen::MatrixXd::Identity(5,5);

	// Propagation Step
	xk = xk + xd*(actualIter-previousIter).toSec();
	pk = fk*pk*fk.transpose() + qq;

	previousIter = actualIter;
}

void EKF::correct(Eigen::Matrix<double, 3, 1> zk, Eigen::Matrix<double, 3, 3> rk){
	if(!ready){
		ROS_WARN("[EKF]: Initial Condition Required, Please Perform Reset");
		return;
	}

	actualIter = ros::Time::now();

	Eigen::Matrix<double, 5, 1> xd;
	Eigen::Matrix<double, 5, 5> fk;
	Eigen::Matrix<double, 3, 1> hk;

	// Propagate Continuous-Time Dynamics
	getSystemDynamics(xd, xk);

	// Jacobian
	getJacobian(fk, xk);
	fk = fk*(actualIter-previousIter).toSec() + Eigen::MatrixXd::Identity(5,5);

	// Propagation Step
	xk = xk + xd*(actualIter-previousIter).toSec();
	pk = fk*pk*fk.transpose() + qq;

	previousIter = actualIter;

	hk(0) = xk(0);
	hk(1) = xk(1);
	hk(2) = xk(3);

	// Update Step
	Eigen::Matrix<double, 3, 1> yk = zk - hk;
	Eigen::Matrix<double, 5, 3> kk = pk*(hh.transpose())*(hh*pk*hh.transpose() + rk).inverse();

	xk = xk + (kk*yk);
	pk = (Eigen::MatrixXd::Identity(5,5) - kk*hh)*pk;
}