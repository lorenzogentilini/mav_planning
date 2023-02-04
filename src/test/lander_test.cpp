#include <mav_planning/lander.hpp>

using namespace _Lander_;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_lander");
	ros::NodeHandle nh("~");

  	ros::Rate rate(5.0);

	Environment* environment = new Environment(nh);
	Lander lander(nh, environment);

	while(ros::ok()){
		std::vector<Eigen::Matrix<double, 3, 1>> cp;
		double tt;
		lander.land(cp, tt);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}