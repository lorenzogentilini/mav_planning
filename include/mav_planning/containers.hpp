#pragma once
#include <ros/ros.h>

#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <mav_planning/rpoly.hpp>

template <typename T>
class BSpline{
	private:
	int spline_order;
	
	bool vv = false;
	bool aa = false;
	bool jj = false;

	std::vector<T> cp;
	std::vector<T> vp;
	std::vector<T> ap;
	std::vector<T> jp;
	std::vector<T> wp;

	std::vector<double> pk;
	std::vector<double> vk;
	std::vector<double> ak;
	std::vector<double> jk;

	Eigen::MatrixXd MB;
	Eigen::MatrixXd MB_inv;
	std::vector<Eigen::MatrixXd> MSs;
	std::vector<Eigen::MatrixXd> MSs_inv;

	public:
	// Class Constructor
	BSpline(int _spline_order);

	// Class Destructor
	~BSpline();

	void setControl(std::vector<T> _cp, double tt);
	void setControl(std::vector<T> _cp, std::vector<double> kk);
	void computeDerivate();
	void setWaypoints(std::vector<T> _wp, std::vector<std::vector<T>> bb, double tt);
	void fromBezier(std::vector<std::vector<T>> _cp, double tt);
	void toBezier(std::vector<std::vector<T>>& _cp, double& tt);
	double deBoor(int n, int k, double time, std::vector<double> knot);
	T evaluateCurve(double tt, int dd);
	std::vector<T> getControl();
	std::vector<double> getKnots();
	double getTime();
	T getTotalJerk();
	void clearAll();
	void computeBezierMatrix();
	void computeSplineMatrix();
	void insertSpline(std::vector<T> _cp, double ti, double tf, double dt);
	std::vector<double> generateHeadingTrajectory();
	bool checkType();

	double to_2PI(double yy){
		while(yy < 0){
			yy = yy + 2*M_PI;
		}
		
		while(yy >= 2*M_PI){
			yy = yy - 2*M_PI;
		}

		return yy;
	};

	double to_PI(double yy){
		while(yy < -M_PI){
			yy = yy + M_PI;
		}
		
		while(yy > M_PI){
			yy = yy - M_PI;
		}

		return yy;
	};
};

// Constructor
template <typename T>
BSpline<T>::BSpline(int _spline_order){
	spline_order = _spline_order;
	computeBezierMatrix();

	if(spline_order+1 >= 4){
		vv = true;
		if(spline_order+1 >= 6){
			aa = true;
			if(spline_order+1 >= 8){
				jj = true;
			}
		}
	}
}

// Destructor
template <typename T>
BSpline<T>::~BSpline(){
	;
}

template <typename T>
void BSpline<T>::insertSpline(std::vector<T> _cp, double ti, double tf, double dt){
	double tt = pk[pk.size()-1];
	double dt_tt = tt/(cp.size() - spline_order);

	// int nn_cp_init = cp.size();

	std::vector<T> new_cp;
	int nn = ceil((spline_order+1)/2);

	// Search ti IDX
	int ti_idx = 0;
	for(uint ii = spline_order; ii < pk.size(); ii++){
		if(pk[ii] >= ti){
			ti_idx = ii-spline_order-1;
			break;
		}
	}

	// Search tf IDX
	int tf_idx = 0;
	for(uint ii = spline_order; ii < pk.size(); ii++){
		if(pk[ii] >= tf){
			tf_idx = ii-spline_order-1;
			break;
		}
	}

	for(uint ii = 0; ii < nn+ti_idx; ii++){
		new_cp.push_back(cp[ii]);
	}

	new_cp.insert(new_cp.end(), _cp.begin(), _cp.end());

	for(uint ii = nn+tf_idx; ii < cp.size(); ii++){
		new_cp.push_back(cp[ii]);
	}

	// int nn_cp_diff = new_cp.size() - nn_cp_init;

	// setControl(new_cp, ti+dt+(tt-tf)); //TODO
	setControl(new_cp, dt_tt*(new_cp.size() - spline_order));
}

template <typename T>
std::vector<double> BSpline<T>::generateHeadingTrajectory(){
	if(!checkType()){
		ROS_ERROR("[CONTAINER]: Impossible to Generate Heading");
		exit(-1);
	}

	std::vector<T> wp_cc;
	std::vector<T> dp_cc;
	int wp_size = cp.size() - (spline_order+1);
	double dt = getTime()/wp_size;
	for(int ii = 0; ii <= wp_size; ii++){
		T pp = evaluateCurve(dt*ii, 0);
		wp_cc.push_back(pp);
	}

	for(int ii = 1; ii <= wp_size; ii++){
		T dp = wp_cc[ii]-wp_cc[ii-1];
		dp_cc.push_back(dp);
	}
	
	BSpline<double> curve_hh(3);
	std::vector<double> wp_hh;
	std::vector<double> vv;
	std::vector<std::vector<double>> bb;

	vv.insert(vv.begin(), 2, 0.0);
	bb.push_back(vv);

	wp_hh.push_back(to_2PI(atan2(dp_cc.front()(1), dp_cc.front()(0))));
	for(int ii = 1; ii < dp_cc.size(); ii++){
		double yy = to_2PI(atan2(dp_cc[ii](1), dp_cc[ii](0)));
		double prev_yy = to_2PI(wp_hh.back());

		double dy = yy - prev_yy;
		double dy_2PI = abs(dy) - 2*M_PI;

		if(dy < 0){
			dy_2PI = -dy_2PI;
		}

		dy = abs(dy) < abs(dy_2PI) ? dy : dy_2PI;
		wp_hh.push_back(wp_hh.back() + dy);
	}

	// for(uint ii = 0; ii < wp_hh.size(); ii++){
	// 	wp_hh[ii] = to_PI(wp_hh[ii]);
	// }

	curve_hh.setWaypoints(wp_hh, bb, getTime());
	return curve_hh.getControl();
}

template <typename T>
bool BSpline<T>::checkType(){
	if(std::is_same<T, Eigen::Matrix<double, 3, 1>>::value ||
	   std::is_same<T, Eigen::Matrix<double, 1, 3>>::value ||
	   std::is_same<T, Eigen::Matrix<double, 2, 1>>::value ||
	   std::is_same<T, Eigen::Matrix<double, 1, 2>>::value ||
	   std::is_same<T, Eigen::Matrix<float,  3, 1>>::value ||
	   std::is_same<T, Eigen::Matrix<float,  1, 3>>::value ||
	   std::is_same<T, Eigen::Matrix<float,  2, 1>>::value ||
	   std::is_same<T, Eigen::Matrix<float,  1, 1>>::value){
		return true;
	}
	return false;
}

template <typename T>
void BSpline<T>::setControl(std::vector<T> _cp, double tt){
	clearAll();
	cp.resize(_cp.size());
	cp = _cp;

	// Generate Equally Spaced Knots
	double dt = tt/(cp.size() - spline_order);
	pk.resize(cp.size() + (spline_order+1));
	for(uint ii = 0; ii < cp.size() + (spline_order+1); ii++){
		if(ii <= spline_order)
			pk[ii] = 0;
		else if(ii > pk.size() - (spline_order+1))
			pk[ii] = pk[ii-1];
		else
			pk[ii] = pk[ii-1] + dt;
	}

	computeDerivate();
}

template <typename T>
void BSpline<T>::computeDerivate(){
	if(vv){
		// Generate Velocity Knots
		vk.resize(pk.size()-2);
		for(uint ii = 1; ii < pk.size()-1; ii++)
			vk[ii-1] = pk[ii];

		// Compute Velocity Control Points
		vp.resize(cp.size()-1);
		for(uint ii = 0; ii < cp.size()-1; ii++){
			double dk = pk[ii+spline_order+1] - pk[ii+1];
			vp[ii] = (cp[ii+1] - cp[ii])*(spline_order/dk);
		}
	}

	if(aa){
		// Generate Acceleration Knots
		ak.resize(vk.size()-2);
		for(uint ii = 1; ii < vk.size()-1; ii++)
			ak[ii-1] = vk[ii];

		// Compute Acceleration Control Points
		ap.resize(vp.size()-1);
		for(uint ii = 0; ii < vp.size()-1; ii++){
			double dk = vk[ii+spline_order] - vk[ii+1];
			ap[ii] = (vp[ii+1] - vp[ii])*((spline_order-1)/dk);
		}
	}

	if(jj){
		// Generate Jerk Knots
		jk.resize(ak.size()-2);
		for(uint ii = 1; ii < ak.size()-1; ii++)
			jk[ii-1] = ak[ii];

		// Compute Jerk Control Points
		jp.resize(ap.size()-1);
		for(uint ii = 0; ii < ap.size()-1; ii++){
			double dk = ak[ii+spline_order-1] - ak[ii+1];
			jp[ii] = (ap[ii+1] - ap[ii])*((spline_order-2)/dk);
		}
	}

	computeSplineMatrix();
}

template <typename T>
void BSpline<T>::setControl(std::vector<T> _cp, std::vector<double> kk){
	clearAll();

	// Control Points Assignment
	cp.resize(_cp.size());
	cp = _cp;

	// Knots Assignment
	pk.resize(kk.size());
	pk = kk;

	computeDerivate();
}

template <typename T>
void BSpline<T>::setWaypoints(std::vector<T> _wp, std::vector<std::vector<T>> bb, double tt){
	clearAll();

	if(spline_order-1 < 2*bb.size()){
		ROS_ERROR("[BSPLINE]: Spline Order Too Low!!");
		return;
	}

	int cp_size = _wp.size() + 2*bb.size();
	double dk = tt/(cp_size - spline_order);
	double ts = tt/(_wp.size()-1);

	// Generate Equally Spaced Knots
	pk.resize(cp_size + (spline_order+1));
	for(uint ii = 0; ii < pk.size(); ii++){
		if(ii <= spline_order)
			pk[ii] = 0;
		else if(ii > pk.size() - (spline_order+1))
			pk[ii] = pk[ii-1];
		else
			pk[ii] = pk[ii-1] + dk;
	}

	// Fill Matrix A
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(cp_size, cp_size);
	for(uint ii = 0; ii < _wp.size(); ii++){
		for(uint kk = 0; kk < cp_size; kk++){
			double t = ts*ii;
			if(t >= pk[pk.size()-1])
				t = pk[pk.size()-1] - 1e-10;

			A(ii, kk) = deBoor(spline_order, kk, t, pk);
		}
	}

	int idx_v = 0, idx_a = 0, idx_j = 0;
	int entryPoint_actual = _wp.size();
	if(bb.size() > 0){ // Velocity Constraints
		// Generate Velocity Knots
		vk.resize(pk.size()-2);
		for(uint ii = 1; ii < pk.size()-1; ii++){
			vk[ii-1] = pk[ii];
		}

		idx_v = 0;
		double v_init_1 = spline_order/(pk[idx_v+spline_order+1] - pk[idx_v+1]);
		idx_v = cp_size-2;
		double v_end_1 = spline_order/(pk[idx_v+spline_order+1] - pk[idx_v+1]);

		Eigen::Matrix<double, 1, 2> velocitySelector(-1, 1);
		A.block(entryPoint_actual, 0, 1, 2) = v_init_1*velocitySelector;
		A.block(entryPoint_actual+1, cp_size-2, 1, 2) = v_end_1*velocitySelector;
		entryPoint_actual += 2;

		if(bb.size() > 1){ // Acceleration Constraints
			// Generate Acceleration Knots
			ak.resize(vk.size()-2);
			for(uint ii = 1; ii < vk.size()-1; ii++){
				ak[ii-1] = vk[ii];
			}

			idx_v = 1;
			double v_init_2 = spline_order/(pk[idx_v+spline_order+1] - pk[idx_v+1]);
			idx_v = cp_size-3;
			double v_end_2 = spline_order/(pk[idx_v+spline_order+1] - pk[idx_v+1]);

			idx_a = 0;
			double a_init_1 = (spline_order-1)/(vk[idx_a+spline_order] - vk[idx_a+1]);
			idx_a = cp_size-3;
			double a_end_1 = (spline_order-1)/(vk[idx_a+spline_order] - vk[idx_a+1]);

			Eigen::Matrix<double, 1, 3> accelerationSelector_init(v_init_1, -(v_init_1+v_init_2), v_init_2);
			Eigen::Matrix<double, 1, 3> accelerationSelector_end(v_end_2, -(v_end_1+v_end_2), v_end_1);
			A.block(entryPoint_actual, 0, 1, 3) = a_init_1*accelerationSelector_init;
			A.block(entryPoint_actual+1, cp_size-3, 1, 3) = a_end_1*accelerationSelector_end;
			entryPoint_actual += 2;

			if(bb.size() > 2){
				// Generate Jerk Knots
				jk.resize(ak.size()-2);
				for(uint ii = 1; ii < ak.size()-1; ii++){
					jk[ii-1] = ak[ii];
				}

				idx_v = 2;
				double v_init_3 = spline_order/(pk[idx_v+spline_order+1] - pk[idx_v+1]);
				idx_v = cp_size-4;
				double v_end_3 = spline_order/(pk[idx_v+spline_order+1] - pk[idx_v+1]);

				idx_a = 1;
				double a_init_2 = (spline_order-1)/(vk[idx_a+spline_order] - vk[idx_a+1]);
				idx_a = cp_size-4;
				double a_end_2 = (spline_order-1)/(vk[idx_a+spline_order] - vk[idx_a+1]);

				idx_j = 0;
				double j_init_1 = (spline_order-2)/(ak[idx_j+spline_order-1] - ak[idx_j+1]);
				idx_j = cp_size-4;
				double j_end_1 = (spline_order-2)/(ak[idx_j+spline_order-1] - ak[idx_j+1]);

				Eigen::Matrix<double, 1, 4> jerkSelector_init(-v_init_1*a_init_1, v_init_2*a_init_2 + v_init_2*a_init_1 + v_init_1*a_init_1,
															  -v_init_3*a_init_2 - v_init_2*a_init_2 - v_init_2*a_init_1, v_init_3*a_init_2);
				Eigen::Matrix<double, 1, 4> jerkSelector_end(-v_end_3*a_end_2, v_end_2*a_end_1 + v_end_2*a_end_2 + v_end_3*a_end_2,
															 -v_end_1*a_end_1 - v_end_2*a_end_1 - v_end_2*a_end_2, v_end_1*a_end_1);
				A.block(entryPoint_actual, 0, 1, 4) = j_init_1*jerkSelector_init;
				A.block(entryPoint_actual+1, cp_size-4, 1, 4) = j_end_1*jerkSelector_end;
				entryPoint_actual += 2;
			}
		}
	}
	
	// Solve Ax = b
	Eigen::MatrixXd A_inv = A.inverse();

	cp.resize(cp_size);
	for(uint ii = 0; ii < cp_size; ii++){
		T actual_cp;
		for(uint jj = 0; jj < _wp.size(); jj++){
			if(jj == 0){
				actual_cp = A_inv(ii,jj)*_wp[jj];
			} else{
				actual_cp += A_inv(ii,jj)*_wp[jj];
			}
		}

		for(uint jj = 0; jj < bb.size(); jj++){
			actual_cp += A_inv(ii,(jj*2)+_wp.size())*bb[jj][0];
			actual_cp += A_inv(ii,(jj*2)+1+_wp.size())*bb[jj][1];
		}

		cp[ii] = actual_cp;
	}

	computeDerivate();
}

template <typename T>
void BSpline<T>::fromBezier(std::vector<std::vector<T>> _cp, double tt){
	clearAll();

	// Generate Equally Spaced Knots
	cp.resize(_cp.size() + spline_order);
	double dt = tt/(cp.size() - spline_order);
	pk.resize(cp.size() + (spline_order+1));
	for(uint ii = 0; ii < cp.size() + (spline_order+1); ii++){
		if(ii <= spline_order)
			pk[ii] = 0;
		else if(ii > pk.size() - (spline_order+1))
			pk[ii] = pk[ii-1];
		else
			pk[ii] = pk[ii-1] + dt;
	}

	computeSplineMatrix();

	for(uint ii = 0; ii < MSs_inv.size(); ii++){
		Eigen::MatrixXd h = MSs_inv[ii]*MB;

		for(uint jj = 0; jj < h.rows(); jj++){
			T v;
			for(uint kk = 0; kk < h.cols(); kk++){
				T v_ = h(jj, kk)*_cp[ii][kk];

				if(kk == 0){
					v = v_;
				} else{
					v = v + v_;
				}
			}

			if(ii < MSs_inv.size()-1){
				if(jj == 0){
					cp[ii] = v;
				}
			}

			if(ii == MSs_inv.size()-1){
				cp[ii+jj] = v;
			}
		}
	}

	computeDerivate();
}

template <typename T>
void BSpline<T>::toBezier(std::vector<std::vector<T>>& _cp, double& tt){
	std::vector<std::vector<T>> _cp_;

	uint n = pk.size() - cp.size() - 1;
	for(uint ii = 0; ii < MSs.size(); ii++){
		Eigen::MatrixXd h = MB_inv*MSs[ii];
		std::vector<T> cp_accumulator;

		for(uint jj = 0; jj < h.rows(); jj++){
			T v;
			for(uint kk = 0; kk < h.cols(); kk++){
				T v_ = h(jj, kk)*cp[kk + ii];

				if(kk == 0){
					v = v_;
				} else{
					v = v + v_;
				}
			}

			cp_accumulator.push_back(v);
		}

		_cp_.push_back(cp_accumulator);
	}

	_cp = _cp_;
	tt = pk[pk.size()-1];
}

template <typename T>
void BSpline<T>::computeSplineMatrix(){
	uint n = pk.size() - cp.size() - 1;
	uint N = cp.size() - 1;

	for(uint ti = n; ti < N+1; ti++){
		Eigen::MatrixXd m(1,1);
		m(0,0) = 1.0;

		for(uint ii = 1; ii < n+1; ii++){
			Eigen::MatrixXd mk = Eigen::MatrixXd::Zero(2*m.rows() -(ii-1), ii+1);
			Eigen::MatrixXd d = Eigen::MatrixXd::Zero(ii, ii+1);
			Eigen::MatrixXd dd = Eigen::MatrixXd::Zero(ii, ii+1);

			for(uint jj = 0; jj < ii; jj++){
				for(uint kk = 0; kk < ii+1; kk++){
					double den = pk[ti -(ii+1)+2+jj + (ii)] - pk[ti - (ii+1)+2+jj];

					double dt, ddt;
					if(den == 0){
						dt = 0;
						ddt = 0;
					} else{
						dt = (pk[ti] - pk[ti - (ii+1)+2+jj])/den;
						ddt = (pk[ti+1] - pk[ti])/den;
					}

					if(jj == kk){
						d(jj,kk) = 1-dt;
						dd(jj,kk) = -ddt;
					} else if(jj == kk-1){
						d(jj,kk) = dt;
						dd(jj,kk) = ddt;
					} else{
						d(jj,kk) = 0.0;
						dd(jj,kk) = 0.0;
					}
				}
			}

			Eigen::MatrixXd m_ = m*d;
			Eigen::MatrixXd m__ = m*dd;
			for(uint hh = 0; hh < m_.rows(); hh++){
				for(uint kk = 0; kk < m_.cols(); kk++){
					mk(hh,kk) = m_(hh,kk);
				}
			}
			
			uint tobe_summed = (ii-1);
			for(uint hh = 0; hh < m__.rows(); hh++){
				for(uint kk = 0; kk < m__.cols(); kk++){
					if(hh < tobe_summed){
						mk(m_.rows()-(tobe_summed-hh),kk) += m__(hh,kk);
					} else{
						mk(m_.rows()+(hh-tobe_summed),kk) = m__(hh,kk);
					}
				}
			}

			m.resize(mk.rows(), mk.cols());
			m = mk;
		}

		MSs.push_back(m);
		MSs_inv.push_back(m.inverse());
	}
}

template <typename T>
void BSpline<T>::computeBezierMatrix(){
	MB = Eigen::MatrixXd::Zero(spline_order+1, spline_order+1);
	switch(spline_order){
		case 3:{
			MB(0,0) =  1.0;

			MB(1,0) = -3.0;
			MB(1,1) =  3.0;

			MB(2,0) =  3.0;
			MB(2,1) = -6.0;
			MB(2,2) =  3.0;

			MB(3,0) = -1.0;
			MB(3,1) =  3.0;
			MB(3,2) = -3.0;
			MB(3,3) =  1.0;
			break;
		}

		case 7:{
			MB(0,0) =  1.0;

			MB(1,0) = -7.0;
			MB(1,1) =  7.0;

			MB(2,0) =  21.0;
			MB(2,1) = -42.0;
			MB(2,2) =  21.0;

			MB(3,0) = -35.0;
			MB(3,1) =  105.0;
			MB(3,2) = -105.0;
			MB(3,3) =  35.0;

			MB(4,0) =  35.0;
			MB(4,1) = -140.0;
			MB(4,2) =  210.0;
			MB(4,3) = -140.0;
			MB(4,4) =  35.0;

			MB(5,0) = -21.0;
			MB(5,1) =  105.0;
			MB(5,2) = -210.0;
			MB(5,3) =  210.0;
			MB(5,4) = -105.0;
			MB(5,5) =  21.0;

			MB(6,0) =  7.0;
			MB(6,1) = -42.0;
			MB(6,2) =  105.0;
			MB(6,3) = -140.0;
			MB(6,4) =  105.0;
			MB(6,5) = -42.0;
			MB(6,6) =  7.0;

			MB(7,0) = -1.0;
			MB(7,1) =  7.0;
			MB(7,2) = -21.0;
			MB(7,3) =  35.0;
			MB(7,4) = -35.0;
			MB(7,5) =  21.0;
			MB(7,6) = -7.0;
			MB(7,7) =  1.0;
			break;
		}

		default:{
			ROS_ERROR("[BSPLINE]: Order Not Implemented!!");
			exit(-1);
		}
	}

	MB_inv = MB.inverse();
}

template <typename T>
void BSpline<T>::clearAll(){
	cp.clear();
	vp.clear();
	ap.clear();
	jp.clear();

	pk.clear();
	vk.clear();    
	ak.clear();    
	jk.clear();    

	wp.clear();

	cp.shrink_to_fit();
	vp.shrink_to_fit();
	ap.shrink_to_fit();
	jp.shrink_to_fit();

	pk.shrink_to_fit();
	vk.shrink_to_fit();
	ak.shrink_to_fit();
	jk.shrink_to_fit();

	wp.shrink_to_fit();
}

template <typename T>
T BSpline<T>::evaluateCurve(double tt, int dd){
	std::vector<T> pp;
	std::vector<double> kk;
	switch(dd){
		case 0:{
			pp = cp;
			kk = pk;
			break;
		}

		case 1:{
			pp = vp;
			kk = vk;
			break;
		}

		case 2:{
			pp = ap;
			kk = ak;
			break;
		}

		case 3:{
			pp = jp;
			kk = jk;
			break;
		}

		default:{
			ROS_ERROR("[BSPLINE]: Evaluation Error");
			exit(-1);
			break;
		}
	}

	if(pp.size() == 0){
		ROS_ERROR("[BSPLINE]: Evaluation Error");
		exit(-1);
	}

	double tr = 0.0;
	if(tt >= kk[kk.size()-1] - 1e-3){
		tr = kk[kk.size()-1] - 1e-3;
	} else{
		tr = tt;
	}

	T result;
	for(uint ii = 0; ii < pp.size(); ii++){
		double B = deBoor(spline_order-dd, ii, tr, kk);

		if(ii == 0){
			result = pp[ii]*B;
		} else{
			result += pp[ii]*B;
		}
	}

	return result;
}

template <typename T>
std::vector<T> BSpline<T>::getControl(){
	return cp;
}

template <typename T>
std::vector<double> BSpline<T>::getKnots(){
	return pk;
}

template <typename T>
double BSpline<T>::getTime(){
	return pk[pk.size()-1];
}

template <typename T>
T BSpline<T>::getTotalJerk(){
	T sum = jp[0];
	for(uint ii = 1; ii < jp.size(); ii++){
		sum += jp[ii];
	}

	return sum;
}

template <typename T>
double BSpline<T>::deBoor(int n, int k, double time, std::vector<double> knot){
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

class Polynomial{
	private:
	int poly_order;
	Eigen::VectorXd poly_coeff;
	Eigen::MatrixXd base_coeff;

	Eigen::MatrixXd computeBaseCoefficients();
	bool computeMinMaxCandidates(double st, double et, int dd, std::vector<double>& candidates);
	bool selectCandidatesFromRoots(double st, double et, Eigen::VectorXcd roots, std::vector<double>& candidates);
	bool selectMinMaxFromCandidates(std::vector<double>& candidates, int dd, std::pair<double, double>& min, std::pair<double, double>& max);

	public:
	// Class Constructor
	Polynomial(int _poly_order);
	Polynomial(Eigen::VectorXd coeffs);
	Polynomial(int _poly_order, double st, double et, std::vector<double> stConstraints, std::vector<double> etConstraints);

	// Class Destructor
	~Polynomial();

	void setCoefficients(Eigen::VectorXd coeffs);
	Eigen::VectorXd getCoefficients(uint dd);
	double getBaseCoefficient(int dd, int idx);
	Eigen::VectorXd getBaseWithTime(uint dd, double tt);
	double evaluate(uint dd, double time);
	bool computeMinMax(double st, double et, int dd, std::pair<double, double>& min, std::pair<double, double>& max);
	Eigen::VectorXd convolve(uint dd, uint kernel);
	bool getRoots(int dd, Eigen::VectorXcd& roots);
};