#include <mav_planning/containers.hpp>

int main(int argc, char **argv){
	ros::init(argc, argv, "test_containers");
	ros::NodeHandle nh("~");

	BSpline<Eigen::Matrix<double, 1, 3>> spline(7);

	std::vector<Eigen::Matrix<double, 1, 3>> _wp;
	_wp.push_back(Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0));
	_wp.push_back(Eigen::Matrix<double, 1, 3>(0.5, 0.0, 1.2));
	_wp.push_back(Eigen::Matrix<double, 1, 3>(1.0, 0.0, 1.4));
	_wp.push_back(Eigen::Matrix<double, 1, 3>(1.0, 0.0, 1.6));
	_wp.push_back(Eigen::Matrix<double, 1, 3>(0.5, 0.0, 1.8));
	_wp.push_back(Eigen::Matrix<double, 1, 3>(0.0, 0.0, 3.0));

	std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> _bb;
	std::vector<Eigen::Matrix<double, 1, 3>> _bv;
	std::vector<Eigen::Matrix<double, 1, 3>> _ba;
	std::vector<Eigen::Matrix<double, 1, 3>> _bj;

	_bv.push_back(Eigen::Matrix<double, 1, 3>(1.0, 0.5, 0.1));
	_bv.push_back(Eigen::Matrix<double, 1, 3>(0.2, 0.5, 1.0));
	_ba.push_back(Eigen::Matrix<double, 1, 3>(0.1, 0.0, 0.2));
	_ba.push_back(Eigen::Matrix<double, 1, 3>(0.3, 0.2, 1.0));
	_bj.push_back(Eigen::Matrix<double, 1, 3>(2.0, 0.5, 0.2));
	_bj.push_back(Eigen::Matrix<double, 1, 3>(0.5, 0.1, 1.0));

	_bb.push_back(_bv);
	_bb.push_back(_ba);
	_bb.push_back(_bj);

	spline.setWaypoints(_wp, _bb, 5.0);

	// std::vector<Eigen::Matrix<double, 1, 3>> cp = spline.getControl();
	// for(auto cpi:cp){
	//   std::cout << cpi << std::endl;
	// }

	std::cout << "P0: " << spline.evaluateCurve(0.0, 0) << std::endl;
	std::cout << "V0: " << spline.evaluateCurve(0.0, 1) << std::endl;
	std::cout << "A0: " << spline.evaluateCurve(0.0, 2) << std::endl;
	std::cout << "J0: " << spline.evaluateCurve(0.0, 3) << std::endl;
	std::cout << "PF: " << spline.evaluateCurve(5.0, 0) << std::endl;
	std::cout << "VF: " << spline.evaluateCurve(5.0, 1) << std::endl;
	std::cout << "AF: " << spline.evaluateCurve(5.0, 2) << std::endl;
	std::cout << "JF: " << spline.evaluateCurve(5.0, 3) << std::endl;

	std::cout << "---> Convert To Bezier" << std::endl;
	std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> _cpb;
	double tt;
	spline.toBezier(_cpb, tt);
	// for(auto cp:_cpb){
	//   for(auto cpi:cp){
	//     std::cout << cpi << " ";
	//   }
	//   std::cout << std::endl;
	// }

	std::cout << "---> Back To BSpline" << std::endl;
	BSpline<Eigen::Matrix<double, 1, 3>> _spline(7);
	_spline.fromBezier(_cpb, tt);

	std::cout << "P0: " << _spline.evaluateCurve(0.0, 0) << std::endl;
	std::cout << "V0: " << _spline.evaluateCurve(0.0, 1) << std::endl;
	std::cout << "A0: " << _spline.evaluateCurve(0.0, 2) << std::endl;
	std::cout << "J0: " << _spline.evaluateCurve(0.0, 3) << std::endl;
	std::cout << "PF: " << _spline.evaluateCurve(5.0, 0) << std::endl;
	std::cout << "VF: " << _spline.evaluateCurve(5.0, 1) << std::endl;
	std::cout << "AF: " << _spline.evaluateCurve(5.0, 2) << std::endl;
	std::cout << "JF: " << _spline.evaluateCurve(5.0, 3) << std::endl;

	// std::vector<Eigen::Matrix<double, 1, 3>> _cp = _spline.getControl();
	// for(auto cp:_cp){
	//   std::cout << cp << std::endl;
	// }

	return 0;
}