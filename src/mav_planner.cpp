#include <mav_planning/mav_planner.hpp>

// Constructor
MavPlanner::MavPlanner(ros::NodeHandle& nh):
actSubscriber(nh.subscribe("/action", 1, &MavPlanner::actionCallback, this)),
setPointPublisher(nh.advertise<mav_regulator::SetPoint>("/setpoint", 1)),
splinePublisher(nh.advertise<mav_regulator::BSpline>("/spline", 1)),
bezierPublisher(nh.advertise<mav_regulator::BezierMultiArray>("/trajectory", 1)),
statePublisher(nh.advertise<mav_planning::Action>("/actual_action", 1)),
actTimer(nh.createTimer(ros::Duration(0.1), &MavPlanner::exec, this)){

	nh.param("publish_debug_sup", debug_flag, true);
	nh.param("use_replanner", use_replanner, false);
	nh.param("takeoff_altitude", takeoff_altitude, 1.7);
	nh.param("safe_margin", safe_margin, 1.7);
	nh.param("takeoff_velocity", takeoff_velocity, 0.5);
	nh.param("rotation_velocity", rotation_velocity, 0.4);
	nh.param("yaw_threshold", yy_thr, 0.1);
	nh.param("nn_cicles_btw_replanner", nn_cicle_wait, 5);
	nh.param("landing_zz_threshold", zz_eps, 0.1);
	nh.param("landing_time_threshold", tt_delay, 1.0);

	environment = new Environment(nh);
	tracker = new _Tracker_::Tracker(nh, environment);
	explorer = new _Explorer_::Explorer(nh, environment);
	replanner = new _Replanner_::Replanner(nh, environment);
	inspector = new _Inspector_::Inspector(nh, environment);
	planner = new _Planner_::Planner(nh, environment);
	lander = new _Lander_::Lander(nh, environment);

	pp_spline = new BSpline<Eigen::Matrix<double, 3, 1>>(7);
	yy_spline = new BSpline<double>(3);
}

// Destructor
MavPlanner::~MavPlanner(){
	delete environment;
	delete tracker;
	delete explorer;
	delete replanner;
	delete inspector;
	delete planner;
	delete lander;
	delete pp_spline;
	delete yy_spline;
}

bool MavPlanner::readActualOdometry(){
	if(!environment->getLastOdomTransform(bodyToOdom_transformation)){
		return false;
	}

	tf2::Vector3 odomBody_translation = bodyToOdom_transformation.getOrigin();
	tf2::Quaternion odomBody_quaternion = bodyToOdom_transformation.getRotation();
	tf2::Matrix3x3 m(odomBody_quaternion);

	double r, p, y;
	m.getRPY(r,p,y);

	actualPosition(0) = odomBody_translation.x();
	actualPosition(1) = odomBody_translation.y();
	actualPosition(2) = odomBody_translation.z();
	actualOrientation = y;

	return true;
}

void MavPlanner::exec(const ros::TimerEvent& e){
	// Inform Other Nodes About Current Action
	statePublisher.publish(currentAction);

	if(!action_valid){
		return;
	}

	if(!readActualOdometry()){
		return;
	}

	switch(currentAction.action){
		case(mav_planning::Action::NULL_ACTION):{
			ROS_INFO_COND(debug_flag && first_entry, "[MAV PLANNER]: Null Action, Do Nothing");
			first_entry = false;
			
			// Reset State
			currentState = PLAN_ACTION;
			break;
		}

		case(mav_planning::Action::EXECUTED):{
			ROS_INFO_COND(debug_flag && first_entry, "[MAV PLANNER]: Action Executed, Switch To NULL");
			first_entry = false;
			
			// Reset State
			currentState = PLAN_ACTION;
			currentAction.action = mav_planning::Action::NULL_ACTION;
			break;
		}

		case(mav_planning::Action::WAIT_EXECUTION):{
			ROS_INFO_COND(debug_flag && first_entry, "[MAV PLANNER]: Wait for Action Execution");
			first_entry = false;

			if(isActionExecuted()){
				// Reset State
				currentState = PLAN_ACTION;
				currentAction.action = mav_planning::Action::EXECUTED;
			}
			break;
		}

		case(mav_planning::Action::EMERGENCY_STOP):{
			ROS_INFO_COND(debug_flag, "[MAV PLANNER]: Switch to Emergency Stop");

			// Perform STOP(
			if(currentAction.data.size() < 3){
				actualSetPoint = actualPosition;
			} else{
				actualSetPoint(0) = currentAction.data[0];
				actualSetPoint(1) = currentAction.data[1];
				actualSetPoint(2) = currentAction.data[2];
			}

			sendPoint(actualSetPoint(0), actualSetPoint(1), actualSetPoint(2), actualOrientation, 0.5, false);

			// Reset State & Action
			currentState = PLAN_ACTION;
			currentAction.action = mav_planning::Action::NULL_ACTION;
			break;
		}

		case(mav_planning::Action::TAKEOFF):{
			ROS_INFO_COND(debug_flag, "[MAV PLANNER]: Switch to Takeoff");

			// Do a Simple Takeoff
			if(currentAction.data.size() == 0){
				actualSetPoint(0) = actualPosition(0);
				actualSetPoint(1) = actualPosition(1);
				actualSetPoint(2) = actualPosition(2) + takeoff_altitude;
			} else{
				actualSetPoint(0) = actualPosition(0);
				actualSetPoint(1) = actualPosition(1);
				actualSetPoint(2) = actualPosition(2) + currentAction.data[0];
			}

			sendPoint(actualSetPoint(0), actualSetPoint(1), actualSetPoint(2), actualOrientation, takeoff_altitude/takeoff_velocity, true);

			// Set Time
			initialTime = ros::Time::now();

			// Reset Action
			currentState = PLAN_ACTION;
			currentAction.action = mav_planning::Action::WAIT_EXECUTION;
			break;
		}

		case(mav_planning::Action::EXPLORE):{			
			switch(currentState){
				case(PLAN_ACTION):{
					ROS_INFO_COND(debug_flag, "[MAV PLANNER]: Switch to Exploration");

					if(!environment->getLastTransform(odomToMap_transformation)){
						ROS_ERROR("[MAV PLANNER]: Exploration Not Possible");

						triggerStop();
						break;
					}

					double r,p;
					tf2::Quaternion qq = odomToMap_transformation.getRotation();
					tf2::Matrix3x3 mm(qq);
					mm.getRPY(r,p,yy_map);

					// Perform An Exploration Step 
					tf2::Transform bodyToMap_transformation = odomToMap_transformation*bodyToOdom_transformation;
					tf2::Vector3 bodyToMap_translation = bodyToMap_transformation.getOrigin();

					Eigen::Matrix<double, 3, 1> pp;
					pp(0) = bodyToMap_translation.x();
					pp(1) = bodyToMap_translation.y();
					pp(2) = takeoff_altitude;

					checkPointCollision(pp);

					std::vector<Eigen::Matrix<double, 3, 1>> wpi;
					std::vector<Eigen::Matrix<double, 3, 1>> cp;
					double tt;

					explorer->explore(pp.block(0,0,2,1), wpi);

					if(wpi.size() < 2){
						ROS_WARN("[MAV PLANNER]: Explorer Goes Wrong, No Points!");
						
						triggerStop();
						break;
					}

					explorer->fitWithBSpline(cp, tt);

					pp_spline->setControl(cp, tt);
					yy_spline->setControl(pp_spline->generateHeadingTrajectory(), tt);

					// Set Action
					currentState = DO_INIT_ACTION;
					break;
				}

				case(DO_INIT_ACTION):{
					// We Need To Rotate Before Start!
					yy_sp = yy_spline->evaluateCurve(0.0, 0) - yy_map;

					yy_sp = to_PI(yy_sp);
					sendPoint(actualPosition(0), actualPosition(1), actualPosition(2), yy_sp, abs(actualOrientation-yy_sp)/rotation_velocity, false);

					// Set Action
					currentState = WAIT_INIT_ACTION;
					break;
				}

				case(WAIT_INIT_ACTION):{
					if(abs(actualOrientation - yy_sp) < yy_thr){
						// Set Action
						currentState = DO_MAIN_ACTION;
					}
					break;
				}

				case(DO_MAIN_ACTION):{
					sendSpline_withYaw(mav_regulator::BSpline::NEW);
			
					// Set Action
					if(use_replanner){
						replanner->setTrajectory(pp_spline->getControl(), pp_spline->getTime());
						currentState = SUPERVISE_ACTION;

					} else{
						currentState = PLAN_ACTION;
						currentAction.action = mav_planning::Action::WAIT_EXECUTION;
					}

					initialTime = ros::Time::now();
					break;
				}

				case(SUPERVISE_ACTION):{
					double dt = (ros::Time::now() - initialTime).toSec();
					superviseAction(dt);

					if(isActionExecuted()){
						// Reset State
						currentState = PLAN_ACTION;
						currentAction.action = mav_planning::Action::EXECUTED;
					}
					break;
				}
			}
			break;
		}

		case(mav_planning::Action::TRACK):{
			ROS_INFO_COND(debug_flag && first_entry, "[MAV PLANNER]: Switch to Tracking");
			first_entry = false;

			std::vector<Eigen::Matrix<double, 3, 1>> cp;
			std::vector<double> cp_yy;
			double dt = (ros::Time::now() - initialTime).toSec();
			
			double tt;
			if(!tracker->track(dt, cp, cp_yy, tt)){
				ROS_ERROR("[MAV PLANNER]: Tracking Impossible");

				triggerStop();
				break;
			}
			
			if(cp.size() != 0){
				pp_spline->setControl(cp, tt);
				yy_spline->setControl(cp_yy, tt);

				initialTime = ros::Time::now();
				sendSpline_withYaw(mav_regulator::BSpline::NEW);
			}

			break;
		}

		case(mav_planning::Action::GOTO):{
			switch(currentState){
				case(PLAN_ACTION):{
					ROS_INFO_COND(debug_flag, "[MAV PLANNER]: Switch to Goto");
					if(currentAction.data.size() < 3){
						ROS_ERROR("[MAV PLANNER]: Called Planner Without Arguments");

						triggerStop();
						break;
					}

					if(!environment->getLastTransform(odomToMap_transformation)){
						ROS_ERROR("[MAV PLANNER]: Planning Not Possible");

						triggerStop();
						break;
					}

					double r,p;
					tf2::Quaternion qq = odomToMap_transformation.getRotation();
					tf2::Matrix3x3 mm(qq);
					mm.getRPY(r,p,yy_map);

					tf2::Transform bodyToMap_transformation = odomToMap_transformation*bodyToOdom_transformation;
					tf2::Vector3 bodyToMap_translation = bodyToMap_transformation.getOrigin();

					Eigen::Matrix<double, 3, 1> ss_;
					Eigen::Matrix<double, 3, 1> gg_;

					ss_(0) = bodyToMap_translation.x();
					ss_(1) = bodyToMap_translation.y();
					ss_(2) = bodyToMap_translation.z();

					gg_(0) = currentAction.data[0];
					gg_(1) = currentAction.data[1];
					gg_(2) = currentAction.data[2];

					checkPointCollision(ss_);
					checkPointCollision(gg_);

					Eigen::Matrix<double, 6, 1> ss;
					Eigen::Matrix<double, 6, 1> gg;

					ss(0) = ss_(0);
					ss(1) = ss_(1);
					ss(2) = ss_(2);
					ss(3) = 0.0;
					ss(4) = 0.0;
					ss(5) = 0.0;

					gg(0) = gg_(0);
					gg(1) = gg_(1);
					gg(2) = gg_(2);
					gg(3) = 0.0;
					gg(4) = 0.0;
					gg(5) = 0.0;

					if(planner->search(ss, gg) != GOAL_REACHED){
						ROS_ERROR("[MAV PLANNER]: Planner Failed");
						
						planner->reset();
						triggerStop();
						break;
					}

					double tt;
					std::vector<Eigen::Matrix<double, 3, 1>> cp;

					planner->getTrajectory(cp, tt);

					pp_spline->setControl(cp, tt);
					yy_spline->setControl(pp_spline->generateHeadingTrajectory(), tt);

					planner->reset();

					currentState = DO_INIT_ACTION;
					break;
				}

				case(DO_INIT_ACTION):{
					// We Need To Rotate Before Start
					yy_sp = yy_spline->evaluateCurve(0.0, 0) - yy_map;

					yy_sp = to_PI(yy_sp);
					sendPoint(actualPosition(0), actualPosition(1), actualPosition(2), yy_sp, abs(actualOrientation-yy_sp)/rotation_velocity, false);

					// Set Action
					currentState = WAIT_INIT_ACTION;
					break;
				}

				case(WAIT_INIT_ACTION):{
					if(abs(actualOrientation - yy_sp) < yy_thr){
						// Set Action
						currentState = DO_MAIN_ACTION;
					}
					break;
				}

				case(DO_MAIN_ACTION):{
					sendSpline_withYaw(mav_regulator::BSpline::NEW);
			
					// Set Action
					if(use_replanner){
						replanner->setTrajectory(pp_spline->getControl(), pp_spline->getTime());
						currentState = SUPERVISE_ACTION;
					} else{
						currentState = PLAN_ACTION;
						currentAction.action = mav_planning::Action::WAIT_EXECUTION;
					}

					initialTime = ros::Time::now();
					break;
				}

				case(SUPERVISE_ACTION):{
					double dt = (ros::Time::now() - initialTime).toSec();
					superviseAction(dt);

					if(isActionExecuted()){
						// Reset State
						currentState = PLAN_ACTION;
						currentAction.action = mav_planning::Action::EXECUTED;
					}
					break;
				}
			}
			break;
		}

		case(mav_planning::Action::INSPECT):{
			switch(currentState){
				case(PLAN_ACTION):{
					ROS_INFO_COND(debug_flag, "[MAV PLANNER]: Switch to Inspection");

					if(currentAction.data.size() < 2){
						ROS_ERROR("[MAV PLANNER]: Called Inspection Without Arguments");

						triggerStop();
						break;
					}

					if(!environment->getLastTransform(odomToMap_transformation)){
						ROS_ERROR("[MAV PLANNER]: Inspection Not Possible");

						triggerStop();
						break;
					}

					double r,p;
					tf2::Quaternion qq = odomToMap_transformation.getRotation();
					tf2::Matrix3x3 mm(qq);
					mm.getRPY(r,p,yy_map);

					tf2::Transform bodyToMap_transformation = odomToMap_transformation*bodyToOdom_transformation;
					tf2::Vector3 bodyToMap_translation = bodyToMap_transformation.getOrigin();

					Eigen::Matrix<double, 2, 1> ip;
					ip(0) = currentAction.data[0];
					ip(1) = currentAction.data[1];

					cp_mm.clear();
					cpy_mm.clear();
					cp_mm.shrink_to_fit();
					cpy_mm.shrink_to_fit();

					inspector->inspect(ip);
					inspector->getTrajectory(cp_mm, cpy_mm, tt_mm);

					if(cp_mm.size() == 0){
						ROS_WARN("[MAV PLANNER]: Inspection Failed, Not Traj.");

						triggerStop();
						break;
					}

					Eigen::Matrix<double, 3, 1> ss_;
					ss_(0) = bodyToMap_translation.x();
					ss_(1) = bodyToMap_translation.y();
					ss_(2) = bodyToMap_translation.z();

					checkPointCollision(ss_);

					Eigen::Matrix<double, 6, 1> ss;
					Eigen::Matrix<double, 6, 1> gg_1;
					Eigen::Matrix<double, 6, 1> gg_2;

					ss(0) = ss_(0);
					ss(1) = ss_(1);
					ss(2) = ss_(2);
					ss(3) = 0.0;
					ss(4) = 0.0;
					ss(5) = 0.0;

					gg_1(0) = cp_mm.front()(0);
					gg_1(1) = cp_mm.front()(1);
					gg_1(2) = cp_mm.front()(2);
					gg_1(3) = 0.0;
					gg_1(4) = 0.0;
					gg_1(5) = 0.0;

					gg_2(0) = cp_mm.back()(0);
					gg_2(1) = cp_mm.back()(1);
					gg_2(2) = cp_mm.back()(2);
					gg_2(3) = 0.0;
					gg_2(4) = 0.0;
					gg_2(5) = 0.0;

					if(planner->search(ss, gg_1) != GOAL_REACHED){
						ROS_ERROR("[MAV PLANNER]: Planner 1 for Inspection Failed");
						planner->reset();

						// Reverse Inspection Trajectory
						std::reverse(cp_mm.begin(), cp_mm.end());
						std::reverse(cpy_mm.begin(), cpy_mm.end());

						if(planner->search(ss, gg_2) != GOAL_REACHED){
							ROS_ERROR("[MAV PLANNER]: Planner 2 for Inspection Failed");
							planner->reset();

							triggerStop();
							break;
						}
					}
					
					double tt;
					std::vector<Eigen::Matrix<double, 3, 1>> cp;

					planner->getTrajectory(cp, tt);

					pp_spline->setControl(cp, tt);
					yy_spline->setControl(pp_spline->generateHeadingTrajectory(), tt);

					planner->reset();

					currentState = DO_INIT_ACTION;
					break;
				}

				case(DO_INIT_ACTION):{
					// We Need To Rotate Before Start
					yy_sp = yy_spline->evaluateCurve(0.0, 0) - yy_map;

					yy_sp = to_PI(yy_sp);
					sendPoint(actualPosition(0), actualPosition(1), actualPosition(2), yy_sp, abs(actualOrientation-yy_sp)/rotation_velocity, false);

					// Set Action
					currentState = WAIT_INIT_ACTION;
					break;
				}

				case(WAIT_INIT_ACTION):{
					if(abs(actualOrientation - yy_sp) < yy_thr){
						// Set Action
						currentState = DO_MAIN_ACTION;
					}
					break;
				}

				case(DO_MAIN_ACTION):{
					sendSpline_withYaw(mav_regulator::BSpline::NEW);

					// Set Action
					if(use_replanner){
						replanner->setTrajectory(pp_spline->getControl(), pp_spline->getTime());
					}
					
					currentState = SUPERVISE_ACTION;
					initialTime = ros::Time::now();
					break;
				}

				case(SUPERVISE_ACTION):{
					double dt = (ros::Time::now() - initialTime).toSec();
					if(use_replanner){
						superviseAction(dt);
					}

					if(dt >= pp_spline->getTime()){
						currentState = DO_SECOND_INIT;
					}
					break;
				}

				case(DO_SECOND_INIT):{
					pp_spline->setControl(cp_mm, tt_mm);
					yy_spline->setControl(cpy_mm, tt_mm);

					// We Need To Rotate Before Start
					yy_sp = yy_spline->evaluateCurve(0.0, 0) - yy_map;

					yy_sp = to_PI(yy_sp);
					sendPoint(actualPosition(0), actualPosition(1), actualPosition(2), yy_sp, abs(actualOrientation-yy_sp)/rotation_velocity, false);

					// Set Action
					currentState = WAIT_SECOND_INIT;
					break;
				}

				case(WAIT_SECOND_INIT):{
					if(abs(actualOrientation - yy_sp) < yy_thr){
						// Set Action
						currentState = DO_SECOND_ACTION;
					}
					break;
				}

				case(DO_SECOND_ACTION):{
					sendSpline_withYaw(mav_regulator::BSpline::NEW);

					// Set Times
					initialTime = ros::Time::now();

					// Reset Action
					currentState = PLAN_ACTION;
					currentAction.action = mav_planning::Action::WAIT_EXECUTION;
				}
			}
			break;
		}

		case(mav_planning::Action::LAND):{
			ROS_INFO_COND(debug_flag && first_entry, "[MAV PLANNER]: Switch to Landing");
			if(first_entry){
				lander->reset();
			}

			first_entry = false;

			std::vector<Eigen::Matrix<double, 3, 1>> cp;
			std::vector<double> cpy;
			double tt;

			if(lander->land(cp, cpy, tt, (ros::Time::now() - initialTime).toSec())){
				if(cp.size() != 0){
					initialTime = ros::Time::now();

					pp_spline->setControl(cp, tt);
					yy_spline->setControl(cpy, tt);
					sendSpline_withYaw(mav_regulator::BSpline::NEW);
				}
			}

			// Check Land Condition
			if(isLanded()){
				ROS_INFO_COND(debug_flag, "[MAV PLANNER]: Landed!!");

				// Reset State
				currentState = PLAN_ACTION;
				currentAction.action = mav_planning::Action::EXECUTED;
			}

			break;
		}

		default:{
			ROS_ERROR("[MAV PLANNER]: Action Not Implemented");
			break;
		}
	}
}

void MavPlanner::superviseAction(double dt){
	if(waitCount < nn_cicle_wait){
		waitCount ++;
		return;
	} else{
		waitCount = 0;
	}

	switch(replanner->superviseTrajectory(dt)){
		case(_Replanner_::Replanner::NO_COLLISION):{
			// Everything OK
			break;
		}

		case(_Replanner_::Replanner::COLLISION):{
			// Collision, Trajectory Replanned
			std::vector<Eigen::Matrix<double, 3, 1>> cp;
			double tt;

			replanner->getTrajectory(cp, tt);
			pp_spline->setControl(cp, tt);
			yy_spline->setControl(pp_spline->generateHeadingTrajectory(), tt);

			sendSpline_withYaw(mav_regulator::BSpline::REPLANNED);
			break;
		}

		case(_Replanner_::Replanner::EMERGENCY_COLLISION):{
			// Collision, Replanning Impossible
			if(currentAction.action == mav_planning::Action::EXPLORE){
				// Reset Exploration
				explorer->stepBack();
			}

			// Break!!
			triggerStop();						
			break;
		}
	}
}

void MavPlanner::triggerStop(){
	currentAction.data.clear();
	currentAction.data.shrink_to_fit();
	
	currentAction.data.push_back(actualPosition(0)); // X Coord
	currentAction.data.push_back(actualPosition(1)); // Y Coord
	currentAction.data.push_back(actualPosition(2)); // Z Coord

	currentAction.action = mav_planning::Action::EMERGENCY_STOP;
	currentState = PLAN_ACTION;
}

bool MavPlanner::isLanded(){
	if(!environment->getLastOdomTransform(bodyToOdom_transformation)){
		return false;
	}

	tf2::Vector3 bodyToOdom_translation = bodyToOdom_transformation.getOrigin();
	bool returnValue = false;


	if(isFirstTime_lander){
		isFirstTime_lander = false;
		zz_init = bodyToOdom_translation.z();
	} else{
		if(abs(zz_init - bodyToOdom_translation.z()) < zz_eps){
			if(isFirstTime_timer){
				isFirstTime_timer = false;

				tt_init = ros::Time::now();
			} else{
				if((ros::Time::now() - tt_init).toSec() > tt_delay){
					isFirstTime_lander = true;
					isFirstTime_timer = true;

					returnValue = true;
				}
			}
		} else{
			isFirstTime_lander = true;
			isFirstTime_timer = true;
		}
	}

	return returnValue;
}

bool MavPlanner::isActionExecuted(){
	switch(currentSetPoint){
		case POINT:{
			// Check Position
			if((actualSetPoint - actualPosition).norm() < 0.3){
				return true;
			}

			break;
		}

		case SPLINE:{
			// Check Both Time and Position
			// Check Time
			double elapsedTime = (ros::Time::now() - initialTime).toSec();
			if(elapsedTime < pp_spline->getTime()){
				return false;
			}

			// Check Position
			if(!environment->getLastTransform(odomToMap_transformation)){
				return false;
			}

			if(!environment->getLastOdomTransform(bodyToOdom_transformation)){
				return false;
			}

			tf2::Transform bodyToMap_transformation = odomToMap_transformation*bodyToOdom_transformation;
			tf2::Vector3 bodyToMap_translation = bodyToMap_transformation.getOrigin();

			Eigen::Matrix<double, 3, 1> pp;
			pp(0) = bodyToMap_translation.x();
			pp(1) = bodyToMap_translation.y();
			pp(2) = bodyToMap_translation.z();

			if((pp - pp_spline->evaluateCurve(pp_spline->getTime(), 0)).norm() < 0.2){
				return true;
			}

			break;
		}
	}

	return false;
}

void MavPlanner::sendSpline(int act){
	mav_regulator::BSpline spline;
	spline.action = act;
	spline.frame = "map";
	spline.is_yawFollowing = true;
	spline.reset = false;

	std::vector<double> kk = pp_spline->getKnots();
	std::vector<Eigen::Matrix<double, 3, 1>> cp = pp_spline->getControl();
	for(uint ii = 0; ii < cp.size(); ii++){
		mav_regulator::Point pp;
		pp.x = cp[ii](0);
		pp.y = cp[ii](1);
		pp.z = cp[ii](2);

		spline.p_controlPoints.push_back(pp);
	}

	spline.p_knots.insert(spline.p_knots.end(), kk.begin(), kk.end());
	splinePublisher.publish(spline);
	currentSetPoint = SPLINE;
}

void MavPlanner::sendSpline_withYaw(int act){
	mav_regulator::BSpline spline;
	spline.action = act;
	spline.frame = "map";
	spline.is_yawFollowing = false;
	spline.reset = false;

	std::vector<double> kk = pp_spline->getKnots();
	std::vector<double> kk_yy = yy_spline->getKnots();
	std::vector<double> cp_yy = yy_spline->getControl();
	std::vector<Eigen::Matrix<double, 3, 1>> cp = pp_spline->getControl();
	for(uint ii = 0; ii < cp.size(); ii++){
		mav_regulator::Point pp;
		pp.x = cp[ii](0);
		pp.y = cp[ii](1);
		pp.z = cp[ii](2);

		spline.p_controlPoints.push_back(pp);
	}

	spline.y_controlPoints.insert(spline.y_controlPoints.end(), cp_yy.begin(), cp_yy.end());
	spline.y_knots.insert(spline.y_knots.end(), kk_yy.begin(), kk_yy.end());
	spline.p_knots.insert(spline.p_knots.end(), kk.begin(), kk.end());
	splinePublisher.publish(spline);
	currentSetPoint = SPLINE;
}

void MavPlanner::sendPoint(double x, double y, double z, double yy, double tt, bool reset){
	mav_regulator::SetPoint sp;
	sp.action = mav_regulator::SetPoint::USE_POSITION;
	sp.x = x;
	sp.y = y;
	sp.z = z;
	sp.yaw = yy;
	sp.time = tt;
	sp.frame = "odom";

	sp.reset = reset;

	setPointPublisher.publish(sp);
	currentSetPoint = POINT;
}

void MavPlanner::checkPointCollision(Eigen::Matrix<double, 3, 1>& pp){
	Eigen::Matrix<double, 3, 1> grad;
	double dd = 0.0;
	
	dd = environment->evaluateSDF_withGradient(pp(0), pp(1), pp(2), grad);
	if(dd < safe_margin){
		grad(2) = 0.0;
		pp += grad*abs(dd - (safe_margin*1.1));
	} else{
		return;
	}

	// Re-Check Collision
	Eigen::Matrix<double, 3, 1> pp_ = pp;
	double max_d = safe_margin*2;

	dd = environment->evaluateSDF(pp_(0), pp_(1), pp_(2));
	while(dd < safe_margin){
		double dx = (((double)std::rand()/(double)RAND_MAX)*2*max_d) - max_d;
		double dy = (((double)std::rand()/(double)RAND_MAX)*2*max_d) - max_d;

		pp_(0) = pp(0) + dx;
		pp_(1) = pp(1) + dy;

		dd = environment->evaluateSDF(pp_(0), pp_(1), pp_(2));
	}

	pp = pp_;
}

void MavPlanner::actionCallback(const mav_planning::Action::ConstPtr& msg){
	initialTime = ros::Time::now();
	currentAction = *msg;
	currentState = PLAN_ACTION;

	action_valid = true;
	first_entry = true;
}