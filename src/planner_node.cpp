#include <mav_planning/mav_planner.hpp>

int main(int argc, char **argv){
	std::srand(std::time(nullptr));
	
	ros::init(argc, argv, "planner_node");
	ros::NodeHandle nh("~");

	ros::MultiThreadedSpinner spinner(2);
	MavPlanner planner(nh);

	spinner.spin();
	return 0;
}