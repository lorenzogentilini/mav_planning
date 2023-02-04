#include <mav_planning/planner.hpp>

using namespace _Planner_;

// Constructor
Planner::Planner(ros::NodeHandle& nh, Environment* _env): environment(_env){
	nh.param("max_number_node", max_nodeNum, 2000);
	nh.param("heuristic_weight", lambda, 10.0);
	nh.param("acc_resolution", a_res, 0.25);
	nh.param("minimum_time", tt_min, 0.3);
	nh.param("maximum_time", tt_max, 0.9);
	nh.param("time_resolution", tt_res, 0.3);
	nh.param("time_step_size", dt, 0.01);
	nh.param("time_weight", rho, 2.0);
	nh.param("safe_margin", safe_margin, 0.5);
	nh.param("planner_map_resolution", m_res, 0.2);
	nh.param("goal_threshold", min_tol, 2.0);
	nh.param("max_velocity", v_max, 1.0);
	nh.param("max_acceleration", a_max, 0.5);
	nh.param("delta_sampling", delta_sampling, 0.5);

	nh.param("max_x", m_max_x, 0.0);
	nh.param("max_y", m_max_y, 10.0);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, 0.0);
	nh.param("min_z", m_min_z, 0.0);

	optimizer = new Optimizer(nh, environment);
	trajectory = new BSpline<Eigen::Matrix<double, 3, 1>>(7);

	nodePool.resize(max_nodeNum);
	for(uint ii = 0; ii < nodePool.size(); ii++)
		nodePool[ii] = new Node;

	// Build Vectors of Inputs and Time Intervals
	bool not_zero = true;
	for(double ax = -a_max; ax <= a_max; ax += a_res){
		for(double ay = -a_max; ay <= a_max; ay += a_res){
			for(double az = -a_max; az <= a_max; az += a_res){
				input.push_back(Eigen::Matrix<double, 3, 1> (ax, ay, az));

				if(ax == 0){
					not_zero = false;
				}
			}
		}
	}

	if(not_zero){
		for(double ax = -a_max; ax <= a_max; ax += a_res){
			for(double ay = -a_max; ay <= a_max; ay += a_res){
				input.push_back(Eigen::Matrix<double, 3, 1> (ax, ay, 0.0));
			}
		}

		for(double ay = -a_max; ay <= a_max; ay += a_res){
			for(double az = -a_max; az <= a_max; az += a_res){
				input.push_back(Eigen::Matrix<double, 3, 1> (0.0, ay, az));
			}
		}

		for(double ax = -a_max; ax <= a_max; ax += a_res){
			for(double az = -a_max; az <= a_max; az += a_res){
				input.push_back(Eigen::Matrix<double, 3, 1> (ax, 0.0, az));
			}
		}
	}

	for(double tau = tt_min; tau <= tt_max; tau += tt_res){
		timeInt.push_back(tau);
	}

	openMap.reserve(max_nodeNum);
	closedSet.reserve(max_nodeNum);

	// std::srand(std::time(nullptr));
}

// Destructor
Planner::~Planner(){
	delete optimizer;
	delete trajectory;

	for(uint ii = 0; ii < nodePool.size(); ii++){
		delete nodePool[ii];
	}
}

int Planner::search(Eigen::Matrix<double, 6, 1> _start, Eigen::Matrix<double, 6, 1> _goal){
	goal = _goal;

	Node* currentNode = nodePool[0];
	Node* neighborNode = NULL;
	usedNode++;
			
	double timeToGoal;
	currentNode->nodeParent = currentNode;
	currentNode->state = _start;
	currentNode->index = indexFromPose(_start.head(3));
	currentNode->g_value = 0.0;
	currentNode->f_value = lambda*compute_hValue(_start, _goal, timeToGoal);
			
	openSet.push(currentNode);
	openMap.insert(std::make_pair(currentNode->index, currentNode));
		
	// Search Loop
	while(!openSet.empty()){
		if(global_flag != SEARCHING){
			return global_flag;
		}

		currentNode = openSet.top();
		openSet.pop();
		openMap.erase(currentNode->index);
		expandNode(currentNode);
	}
	
	// OpenSet Empty, But Any Path Has Been Found
	global_flag = OPEN_EMPTY;
	return global_flag;
}

void Planner::expandNode(Node* node){
	// Determine Termination
	if(global_flag == GOAL_REACHED){
		return;
	}

	double time_to_goal;
	if(((node->state.head(3) - goal.head(3)).norm()) <= min_tol){
		retrieveNodePath(node);

		// Compute One Shot Trajectory
		compute_hValue(node->state, goal, time_to_goal);
		if(oneShotTraj(node->state, goal, time_to_goal)){
			global_flag = GOAL_REACHED;
			return;
		} else{
			global_flag = NO_SHOT_TRAJ;
			return;
		}
	}

	// Expand Node
	std::vector<Node*> tmp_expandedNodes;
	tmp_expandedNodes.reserve(max_nodeNum);
	for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iterInput = input.begin();
		iterInput != input.end(); iterInput++){
					
		for(std::vector<double>::iterator iterTime = timeInt.begin();
			iterTime != timeInt.end(); iterTime++){

			// Obtain a New Node
			Eigen::Matrix<double, 6, 1> x_plus;
			statePropagation(node->state, x_plus, *iterInput, *iterTime);
			Eigen::Matrix<int, 3, 1> idx_plus = indexFromPose(x_plus.head(3));
							
			// Check if New Node is Inside the Map
			if(x_plus(0) <= m_min_x || x_plus(0) >= m_max_x ||
			   x_plus(1) <= m_min_y || x_plus(1) >= m_max_y ||
			   x_plus(2) <= m_min_z || x_plus(2) >= m_max_z){
				
				continue; // Node Outside Map -> Jump
			}
					
			// Check if New Node is Already in Closed Set
			if(closedSet.find(idx_plus) != closedSet.end()){
				continue; // The Node Has Already Been Expanded -> Jump
			}
									
			// Check Velocity Feasibility
			if(fabs(x_plus(3)) > v_max || fabs(x_plus(4)) > v_max || fabs(x_plus(5)) > v_max){
				continue; // Velocity Infeasible -> Jump
			}
									
			// Check if Generated Trajectory is Collision Free
			bool collision = false;
			for(double tt = dt; tt <= *iterTime; tt += dt){
				Eigen::Matrix<double, 6, 1> x_tran;
				statePropagation(node->state, x_tran, *iterInput, tt);
						
				if(checkCollision(x_tran.head(3))){
					collision = true;
					break;
				}
			}

			if(collision){
				continue; // Trajectory isn't Collision Free -> Jump
			}

			// Compute New Cost
			double g_valueTmp = ((*iterInput).squaredNorm() + rho)*(*iterTime) + node->g_value;
			double f_valueTmp = g_valueTmp + lambda*compute_hValue(x_plus, goal, time_to_goal);
									
			// Check if New Node has Been Expanded in This Cycle
			bool prune = false;
			for(std::vector<Node*>::iterator tmpIter = tmp_expandedNodes.begin();
				tmpIter != tmp_expandedNodes.end(); tmpIter++){

				Node* expandedNode = *tmpIter;
				if((expandedNode->index - idx_plus).squaredNorm() == 0){
					prune = true;

					if(f_valueTmp < expandedNode->f_value){
						expandedNode->f_value = f_valueTmp;
						expandedNode->g_value = g_valueTmp;
						expandedNode->state = x_plus;
						expandedNode->time = *iterTime;
						expandedNode->input = *iterInput;
					}
					
					break;
				}
			}


			// Check if New Node is Already Present in OpenSet - Add to OpenSet
			if(!prune){
				std::unordered_map<Eigen::Matrix<int, 3, 1>, Node*, matrixHash<Eigen::Matrix<int, 3, 1>>>::const_iterator gotNode = openMap.find(idx_plus);
				if(gotNode == openMap.end()){
					Node* neighborNode = nodePool[usedNode];
					usedNode ++;

					neighborNode->index = idx_plus;
					neighborNode->state = x_plus;
					neighborNode->g_value = g_valueTmp;
					neighborNode->f_value = f_valueTmp;
					neighborNode->nodeParent = node;
					neighborNode->input = *iterInput;
					neighborNode->time = *iterTime;

					openSet.push(neighborNode);
					openMap.insert(std::make_pair(neighborNode->index, neighborNode));
					tmp_expandedNodes.push_back(neighborNode);

					if(usedNode == max_nodeNum){
						global_flag = MEMORY_OUT; // Running Out of Memory
						return;
					}
				} else{
					// Already Present
					if(g_valueTmp < gotNode->second->g_value){
						gotNode->second->g_value = g_valueTmp;
						gotNode->second->f_value = f_valueTmp;
						gotNode->second->input = *iterInput;
						gotNode->second->nodeParent = node;
						gotNode->second->time = *iterTime;
						gotNode->second->state = x_plus;
					}
				}
			}
		}
	}

	// After Expansion Add Node to Closed Set
	closedSet.insert(std::make_pair(node->index, node));
}

double Planner::compute_hValue(Eigen::Matrix<double, 6, 1> xc, Eigen::Matrix<double, 6, 1> xg, double& optimal_time){
	// Cost Parameters
	double c1 = 12*(xg.head(3) - xc.head(3)).squaredNorm();
	double c2 = -12*(xc.tail(3) + xg.tail(3)).dot(xg.head(3) - xc.head(3));
	double c3 = 4*((xc.tail(3)).squaredNorm() + xc.tail(3).dot(xg.tail(3)) + (xg.tail(3)).squaredNorm());

	// Compute Roots of JPrime
	std::vector<double> timeVect = computeRoot(rho, 0.0, -c3, -2*c2, -3*c1);
			
	// Chech the Correct Time
	double tMinimum = (xg.head(3) - xc.head(3)).lpNorm<Eigen::Infinity>()/(v_max*0.8);
	double cost = c1/pow(tMinimum, 3) + c2/pow(tMinimum, 2) + c3/tMinimum + rho*tMinimum;

	double time = tMinimum;
	for(uint i = 0; i < timeVect.size(); i++)
	for(std::vector<double>::iterator iter = timeVect.begin(); iter != timeVect.end(); iter++){
		double actualTime = 1/(*iter);
		if(actualTime >= tMinimum){
			double c = c1/pow(actualTime, 3) + c2/pow(actualTime, 2) + c3/actualTime + rho*actualTime;

			if(c < cost){
				cost = c;
				time = actualTime;
			}
		}
	}
	
	optimal_time = time;
	return cost;
}

std::vector<double> Planner::computeRoot(double a, double b, double c, double d, double e){
	std::vector<double> roots;
	Eigen::VectorXcd rootsVector;
	Eigen::VectorXd coeffs;
	
	coeffs.resize(5);
	coeffs << a, b, c, d, e;
	Polynomial poly(coeffs);
	poly.getRoots(0, rootsVector);

	roots.reserve(rootsVector.size());
	for(uint ii = 0; ii < rootsVector.size(); ii++){
		// Consider Only Real Roots
		if(abs(rootsVector(ii).imag()) > std::numeric_limits<double>::epsilon() ||
		   rootsVector(ii).real() < std::numeric_limits<double>::epsilon()){

			continue;
		}

		roots.push_back(rootsVector(ii).real());
	}

	return roots;
}

bool Planner::checkCollision(Eigen::Matrix<double, 3, 1> p){
	double dd = environment->evaluateSDF(p(0), p(1), p(2));

	if(dd < safe_margin){
		return true;
	}

	return false;
}

bool Planner::oneShotTraj(Eigen::Matrix<double, 6, 1> xc, Eigen::Matrix<double, 6, 1> xg, double optimal_time){
	Eigen::Matrix<double, 2, 2> coeffMat;
	Eigen::Matrix<double, 2, 1> xDelta, yDelta, zDelta;

	coeffMat << -12/(pow(optimal_time, 3)), 6/(pow(optimal_time, 2)), 6/(pow(optimal_time, 2)), -2/optimal_time;
	xDelta << xg(0) - xc(0) - xc(3)*optimal_time, xg(3) - xc(3);
	yDelta << xg(1) - xc(1) - xc(4)*optimal_time, xg(4) - xc(4);
	zDelta << xg(2) - xc(2) - xc(5)*optimal_time, xg(5) - xc(5);

	Eigen::Matrix<double, 2, 1> xAlpha = coeffMat*xDelta;
	Eigen::Matrix<double, 2, 1> yAlpha = coeffMat*yDelta;
	Eigen::Matrix<double, 2, 1> zAlpha = coeffMat*zDelta;
	Eigen::Matrix<double, 3, 4> poseCoeff;
	Eigen::Matrix<double, 3, 3> velCoeff;
	Eigen::Matrix<double, 3, 2> accCoeff;

	poseCoeff << xAlpha(0)/6, xAlpha(1)/2, xc(3), xc(0),
				 yAlpha(0)/6, yAlpha(1)/2, xc(4), xc(1),
				 zAlpha(0)/6, zAlpha(1)/2, xc(5), xc(2);

	velCoeff << xAlpha(0)/2, xAlpha(1), xc(3),
				yAlpha(0)/2, yAlpha(1), xc(4),
				zAlpha(0)/2, zAlpha(1), xc(5);

	accCoeff << xAlpha(0), xAlpha(1),
				yAlpha(0), yAlpha(1),
				zAlpha(0), zAlpha(1);

	// Check Trajectory
	Eigen::Matrix<double, 4, 1> posePoly;
	Eigen::Matrix<double, 3, 1> velPoly;
	Eigen::Matrix<double, 2, 1> accPoly;
	for(double time = 0; time <= optimal_time; time += dt){
		posePoly << pow(time, 3), pow(time, 2), time, 1;
		velPoly << pow(time, 2), time, 1;
		accPoly << time, 1;

		Eigen::Matrix<double, 3, 1> p, a, v;
		p << poseCoeff.row(0)*posePoly, poseCoeff.row(1)*posePoly, poseCoeff.row(2)*posePoly;
		v << velCoeff.row(0)*velPoly, velCoeff.row(1)*velPoly, velCoeff.row(2)*velPoly;
		a << accCoeff.row(0)*accPoly, accCoeff.row(1)*accPoly, accCoeff.row(2)*accPoly;

		if(p(0) < m_min_x || p(0) > m_max_x ||
		   p(1) < m_min_y || p(1) > m_max_y ||
		   p(2) < m_min_z || p(2) > m_max_z){

			return false; // Position Not Allowed
		}

		if(fabs(v(0)) > v_max || fabs(v(1)) > v_max || fabs(v(2)) > v_max){
			//return false; // Velocity Not Allowed
		}

		if(fabs(a(0)) > a_max || fabs(a(1)) > a_max || fabs(a(2)) > a_max){
			//return false; // Acceleration Not Allowed
		}

		if(checkCollision(p)){
			return false; // Collision!
		}
	}
	
	// If all Checks are OK -> Trajectory Found!
	p_coeff = poseCoeff;
	optimal_trajTime = optimal_time;
	return true;
}

void Planner::retrieveNodePath(Node* node){
	Node* currentNode = node;
	nodePath.push_back(currentNode);

	while(currentNode->nodeParent != currentNode){
		currentNode = currentNode->nodeParent;
		nodePath.push_back(currentNode);
	}

	std::reverse(nodePath.begin(), nodePath.end());
}

void Planner::statePropagation(Eigen::Matrix<double, 6, 1>& xi, Eigen::Matrix<double, 6, 1>& xf, Eigen::Matrix<double, 3, 1> u, double tau){  
	Eigen::Matrix<double, 6, 1> integral;
	Eigen::Matrix<double, 6, 6> intConst = Eigen::MatrixXd::Identity(6, 6);

	for(int ii = 0; ii < 3; ++ii){
		intConst(ii, ii + 3) = tau;
	}

	integral.head(3) = 0.5*pow(tau, 2)*u;
	integral.tail(3) = tau*u;

	xf = intConst*xi + integral;
}

double Planner::getTraj(std::vector<Eigen::Matrix<double, 3, 1>>& sampledTraj_, double stepSize){
	std::vector<Eigen::Matrix<double, 3, 1>> sampledTraj;
	Eigen::Matrix<double, 6, 1> stateSample;

	Node* currentNode = nodePath.back();
	double totalTime = 0.0;

	// Fill Vector With A* Trajectory
	double evaluationTime = currentNode->time - stepSize;
	double last_evaluationTime = evaluationTime;
	sampledTraj.push_back(currentNode->state.head(3));
	if(evaluationTime < 0.0){
		currentNode = currentNode->nodeParent;
		last_evaluationTime = evaluationTime;
		evaluationTime = currentNode->time - abs(evaluationTime);
	}

	while(currentNode->nodeParent != currentNode){
		statePropagation(currentNode->nodeParent->state, stateSample, currentNode->input, evaluationTime);
		sampledTraj.push_back(stateSample.head(3));
		totalTime += stepSize;

		last_evaluationTime = evaluationTime;
		evaluationTime -= stepSize;
		if(evaluationTime < 0.0){
			currentNode = currentNode->nodeParent;
			last_evaluationTime = evaluationTime;
			evaluationTime = currentNode->time - abs(evaluationTime);
		}
	}

	std::reverse(sampledTraj.begin(), sampledTraj.end());

	// Fill Vector With OneShot Trajectory
	Eigen::Matrix<double, 4, 1> pPoly;
	Eigen::Matrix<double, 3, 1> p;

	double tau = 0.0;
	for(tau = abs(last_evaluationTime); tau <= optimal_trajTime; tau += stepSize){
		pPoly << pow(tau, 3), pow(tau, 2), tau, 1;
		p << p_coeff.row(0)*pPoly, p_coeff.row(1)*pPoly, p_coeff.row(2)*pPoly;
		sampledTraj.push_back(p);

		totalTime += stepSize;
	}

	sampledTraj_ = sampledTraj;
	return totalTime;
}

Eigen::Matrix<int, 3, 1> Planner::indexFromPose(Eigen::Matrix<double, 3, 1> pt){
	Eigen::Matrix<double, 3, 1> m_origin = Eigen::Matrix<double, 3, 1>(m_min_x, m_min_y, m_min_z);
	return ((pt - m_origin) / m_res).array().floor().cast<int>();
}

void Planner::getTrajectory(std::vector<Eigen::Matrix<double, 3, 1>>& cp_, double& tt_){
	std::vector<Eigen::Matrix<double, 3, 1>> sampled_trajectory;
	double tt = getTraj(sampled_trajectory, delta_sampling);

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> bounds;
	std::vector<Eigen::Matrix<double, 3, 1>> bounds_accumulator;
	Eigen::Matrix<double, 3, 1> zero = Eigen::MatrixXd::Zero(3, 1);

	bounds_accumulator.insert(bounds_accumulator.begin(), 2, zero);
	for(uint ii = 0; ii < 3; ii++){
		bounds.push_back(bounds_accumulator);
	}

	trajectory->setWaypoints(sampled_trajectory, bounds, tt);

	std::vector<Eigen::Matrix<double, 3, 1>> cp = trajectory->getControl();
	optimizer->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE);

	cp_ = cp;
	tt_ = tt;
}

void Planner::reset(){
	std::priority_queue<Node*, std::vector<Node*>, NodeComparator> emptyQueue;
	for(uint ii = 0; ii < nodePool.size(); ii++){
		nodePool[ii]->nodeParent = NULL;
		nodePool[ii]->g_value = INFINITY;
		nodePool[ii]->f_value = INFINITY;
		nodePool[ii]->time = 0.0;
	}
			
	std::swap(openSet, emptyQueue);
	nodePath.clear();
	nodePath.shrink_to_fit();
	openMap.clear();
	closedSet.clear();

	p_coeff = Eigen::MatrixXd::Zero(3, 4);
	optimal_trajTime = 0.0;
	global_flag = SEARCHING;
	usedNode = 0;
}