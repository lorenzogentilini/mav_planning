#include <mav_planning/explorer.hpp>

using namespace _Explorer_;

// Constructor
Explorer::Explorer(ros::NodeHandle& nh, Environment* _env): environment(_env){
	nh.param("safe_margin", safe_margin, 0.5);
	nh.param("takeoff_altitude", flight_altitude, 1.0);
	nh.param("max_radius", max_rr, 4.0);
	nh.param("min_radius", min_rr, 0.5);
	nh.param("max_n_nodes", max_n_nodes, 1500);
	nh.param("n_childrens", n_childrens, 60);
	nh.param("max_x", m_max_x, 0.0);
	nh.param("max_y", m_max_y, 10.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, 0.0);
	nh.param("sampling_res", s_res, 1.5);
	nh.param("max_velocity_explorer", vv_max, 0.4);
	nh.param("publish_debug_exp", debug_flag, true);

	costmap = new CostMapContainer(nh, environment);
	optimizer = new Optimizer(nh, environment);

	optimizer->setVelocityBound(vv_max);
	old_costmap = costmap->copyTo();

	// std::srand(std::time(nullptr));
}

// Destructor
Explorer::~Explorer(){
  	delete costmap;
	delete optimizer;

  	if(nodeTree.size() != 0){
		for(std::vector<Node*>::iterator iter = nodeTree.begin();
			iter != nodeTree.end(); iter ++){

			delete *iter;
		}
	}
}

void Explorer::reset(){
	if(nodeTree.size() != 0){
		for(std::vector<Node*>::iterator iter = nodeTree.begin();
			iter != nodeTree.end(); iter ++){

			delete *iter;
		}

		nodeTree.clear();
		nodeTree.shrink_to_fit();
  	}

	way_points.clear();
  	way_points.shrink_to_fit();

	std::priority_queue<Node*, std::vector<Node*>, NodeComparator> emptyQueue;
  	std::swap(open_set, emptyQueue);
	
	costmap->reset();
	bestNode = NULL;
}

void Explorer::stepBack(){
	if(nodeTree.size() != 0){
		for(std::vector<Node*>::iterator iter = nodeTree.begin();
			iter != nodeTree.end(); iter ++){

			delete *iter;
		}

		nodeTree.clear();
		nodeTree.shrink_to_fit();
  	}

	way_points.clear();
  	way_points.shrink_to_fit();

	std::priority_queue<Node*, std::vector<Node*>, NodeComparator> emptyQueue;
  	std::swap(open_set, emptyQueue);

	costmap->swap(old_costmap);
	bestNode = NULL;
}

void Explorer::explore(Eigen::Matrix<double, 2, 1> pose, std::vector<Eigen::Matrix<double, 3, 1>>& wp){
	old_costmap = costmap->copyTo();

	initializeTree(pose);
	growTree(pose);

	Node* node = bestNode;
	while(node->nodeParent != node){
		Eigen::Matrix<double, 3, 1> pp;
		pp(0) = node->pose(0);
		pp(1) = node->pose(1);
		pp(2) = flight_altitude;

		way_points.push_back(pp);
		node = node->nodeParent;
	}

	Eigen::Matrix<double, 3, 1> pp;
	pp(0) = node->pose(0);
	pp(1) = node->pose(1);
	pp(2) = flight_altitude;
	way_points.push_back(pp);

	std::reverse(way_points.begin(), way_points.end());
	wp = way_points;

	// Memorize Last CostMap
	costmap->swap(bestNode->costmap);
}

void Explorer::fitWithBSpline(std::vector<Eigen::Matrix<double, 3, 1>>& cp, double& tt){
	if(way_points.size() < 2){
		ROS_ERROR("[EXPLORER]: Something Goes Wrong");
		exit(-1);
	}

	double ll = pathLenght();
	tt = ll/vv_max;

	std::vector<Eigen::Matrix<double, 3, 1>> wp;
	discretizePath(wp);

	cp.insert(cp.begin(), 4, wp.front());
	for(uint ii = 0; ii < wp.size(); ii++){
		cp.push_back(wp[ii]);
	}
	cp.insert(cp.end(), 4, wp.back());

	optimizer->setGuidePoints(wp);
	optimizer->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE | Optimizer::GUIDE);
}

void Explorer::discretizePath(std::vector<Eigen::Matrix<double, 3, 1>>& wp){
	Eigen::Matrix<double, 3, 1> pi;
	Eigen::Matrix<double, 3, 1> pf;
	for(uint ii = 0; ii < way_points.size()-1; ii++){
		pi = way_points[ii];
		pf = way_points[ii+1];

		double rr = (pi-pf).norm();
		int nn = ceil(rr/s_res);
		double dr = rr/nn;

		Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();
		for(int jj = 0; jj <= nn; jj++){
			Eigen::Matrix<double, 3, 1> pp = pi + dr*jj*vv;
			wp.push_back(pp);
		}
	}
}

double Explorer::pathLenght(){
	double ll = 0.0;
	for(uint ii = 0; ii < way_points.size()-1; ii++){
		ll += (way_points[ii+1] - way_points[ii]).norm();
	}

	return ll;
}

void Explorer::growTree(Eigen::Matrix<double, 2, 1> pose){
	while(nodeTree.size() < max_n_nodes && open_set.size() > 0){
		Node* node = open_set.top();
		open_set.pop();

		int nn = 0;
		while(nn < n_childrens){
			nn++;

			// Expand Current Node
			double dd = 0.0;
			Eigen::Matrix<double, 2, 1> pp;

			samplePoint(pp, node->pose);
			dd = environment->evaluateSDF(pp(0), pp(1), flight_altitude);
			if(!insideBounds(pp) || dd < safe_margin){
				continue;
			}

			double ig = node->costmap.evaluateCostRadius(pp(0), pp(1));
			double cc = abs((pp-node->pose).norm() - max_rr/2) + 1.0;
			if(isLineCollision(node->pose, pp)){
				continue;
			}

			// Add New Nodes to Queue
			Node* nn_node = new Node();
			nn_node->nodeParent = node;
			nn_node->pose = pp;
			nn_node->cc = cc;
			nn_node->ig = ig;
			nn_node->costmap = node->costmap.copyTo();

			std::vector<Eigen::Matrix<double, 2, 1>> pps;
			getSamples(node->pose, pp, pps);
			nn_node->costmap.updateCostFromPoints(pps, CostMap::DECREASE);
			nn_node->costmap.updateAllCostMap(CostMap::INCREASE);

			if(nn_node->getTotalUtility() > bestNode->getTotalUtility()){
				bestNode = nn_node;
			}

			nodeTree.push_back(nn_node);
			open_set.push(nn_node);
		}
	}
}

void Explorer::initializeTree(Eigen::Matrix<double, 2, 1> pose){
	Node* node = new Node();
	node->pose = pose;
	node->costmap = costmap->copyTo();
	node->costmap.updateCostRadius(pose(0), pose(1), CostMap::DECREASE);

  	// Ensure The Tree is Empty
	if(nodeTree.size() != 0){
		for(std::vector<Node*>::iterator iter = nodeTree.begin();
			iter != nodeTree.end(); iter ++){

			delete *iter;
		}

		nodeTree.clear();
		nodeTree.shrink_to_fit();
	}

	way_points.clear();
  	way_points.shrink_to_fit();

	std::priority_queue<Node*, std::vector<Node*>, NodeComparator> emptyQueue;
  	std::swap(open_set, emptyQueue);

  	nodeTree.push_back(node);
	open_set.push(node);
	bestNode = node;
}

void Explorer::getSamples(Eigen::Matrix<double, 2, 1> pi, Eigen::Matrix<double, 2, 1> pf, std::vector<Eigen::Matrix<double, 2, 1>>& pps){
	double rr = (pi-pf).norm();
	Eigen::Matrix<double, 2, 1> vv = (pf-pi).normalized();

	for(double dr = 0.0; dr < rr; dr += 0.1){
		Eigen::Matrix<double, 2, 1> pp = pi + vv*dr;
		pps.push_back(pp);
	}
}

bool Explorer::isLineCollision(Eigen::Matrix<double, 2, 1> pi, Eigen::Matrix<double, 2, 1> pf){
	double rr = (pi-pf).norm();
	Eigen::Matrix<double, 2, 1> vv = (pf-pi).normalized();

	for(double dr = 0.1; dr < rr; dr += 0.1){
		Eigen::Matrix<double, 2, 1> pp = pi + vv*dr;
		if(environment->evaluateSDF(pp(0), pp(1), flight_altitude) < safe_margin){
			return true;
		}
	}

	return false;
}

void Explorer::samplePoint(Eigen::Matrix<double, 2, 1>& np, Eigen::Matrix<double, 2, 1> pp){
  // Sample Randomly Over a Sphere
  double max_rr_sq = pow(max_rr, 2);
  double min_rr_sq = pow(min_rr, 2);
  
  	// do{
		do{
			np(0) = (((double)std::rand()/(double)RAND_MAX)*2*max_rr) - max_rr;
			np(1) = (((double)std::rand()/(double)RAND_MAX)*2*max_rr) - max_rr;
		} while(np.squaredNorm() > max_rr_sq || np.squaredNorm() < min_rr_sq);

		np = np + pp;
  	// } while(!insideBounds(np));
}

bool Explorer::insideBounds(Eigen::Matrix<double, 2, 1> pp){
  	if(pp(0) > m_max_x || pp(0) < m_min_x){
		return false;
	}
  	if(pp(1) > m_max_y || pp(1) < m_min_y){
		return false;
	}
  	return true;
}

// Constructor
CostMapContainer::CostMapContainer(ros::NodeHandle& nh, Environment* _env): environment(_env),
  	visPublisher(nh.advertise<visualization_msgs::MarkerArray>("/cost_map", 1)),
	pubTimer(nh.createTimer(ros::Duration(0.5), &CostMapContainer::timerPublish, this)){

	nh.param("planner_map_resolution", m_res, 0.2);
	nh.param("takeoff_altitude", flight_altitude, 1.0);
	nh.param("publish_debug_exp", debug_flag, true);
	nh.param("max_x", m_max_x, 0.0);
	nh.param("max_y", m_max_y, 10.0);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, 0.0);
	nh.param("min_z", m_min_z, 0.0);
	nh.param("cost_increasing_time", cc_inc, 1.01);
	nh.param("cost_decreasing_visiting", cc_dec, 0.5);
	nh.param("sensor_radius", rr, 0.5);
	nh.param("color_factor", m_cf, 0.8);

	computeGrid();
}

// Destructor
CostMapContainer::~CostMapContainer(){
	;
}

void CostMapContainer::timerPublish(const ros::TimerEvent& e){
	if(debug_flag){
		publishMap();
	}
}

void CostMapContainer::reset(){
	m_grid.clear();
	m_grid.shrink_to_fit();
  	computeGrid();
}

void CostMapContainer::computeGrid(){
	// Alloc the 2D Grid
	m_gridSize_x = (int)((m_max_x - m_min_x)/m_res);
	m_gridSize_y = (int)((m_max_y - m_min_y)/m_res); 
	m_gridSize   = m_gridSize_x*m_gridSize_y;
	m_gridStep_y = m_gridSize_x;
	m_grid.resize(m_gridSize);

	for(int iy = 0; iy < m_gridSize_y; iy++){
		for(int ix = 0; ix < m_gridSize_x; ix++){
			double x = ix*m_res + m_min_x;
			double y = iy*m_res + m_min_y;
			int index = ix + iy*m_gridStep_y;

			bool is_free = true;
			for(double z = 0.5; z < m_max_z-0.5; z += m_res/2){
				double dd = environment->evaluateSDF(x,y,z);
				if(dd < m_res){
					is_free = false;
					break;
				}
			}

			if(is_free){
				m_grid[index] = 0.50;
			}

			// bool is_area01 = (x > -18 && x < -13) && (y > 1.5 && y < 6);
			// bool is_area02 = (x > -13.5 && x < -8) && (y > 4 && y < 6);
			// bool is_area03 = (x > -9 && x < -4.5) && (y > 2 && y < 8);

			// bool is_area_start = (x > -13.8 && x < -8) && (y > 5.5 && y < 9.2);
			// bool is_area_guide = (x > -13.8 && x < -8) && (y > 5.5 && y < 8);
			
			// if(is_free){
			// 	if (is_area01 || is_area02 || is_area03){
			// 		m_grid[index] = 0.50;
			// 	} else if(is_area_guide){
			// 		m_grid[index] = 0.10;
			// 	} else if(is_area_start){
			// 		m_grid[index] = 0.05;
			// 	} else{
			// 		m_grid[index] = 0.001;
			// 	}
			// } else{
			// 	m_grid[index] = 0;
			// }
		}
	}
}

void CostMapContainer::publishMap(){
	visualization_msgs::MarkerArray points;
	for(int iy = 0; iy < m_gridSize_y; iy++){
		for(int ix = 0; ix < m_gridSize_x; ix++){
			double x = ix*m_res + m_min_x;
			double y = iy*m_res + m_min_y;
			int index = ix + iy*m_gridStep_y;

			geometry_msgs::Point pp;
			pp.x = x;
			pp.y = y;
			pp.z = flight_altitude;

			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "ns";
			point.id = ix + iy*m_gridStep_y;
			point.type = visualization_msgs::Marker::SPHERE_LIST;
			point.points.push_back(pp);
			point.scale.x = 0.15;
			point.scale.y = 0.15;
			point.scale.z = 0.15;
			point.color.a = 1.0;

			Eigen::Matrix<double, 4, 1> cc = heightColor((1.0 - m_grid[index])*m_cf);
			point.color.r = cc(1);
			point.color.g = cc(2);
			point.color.b = cc(3);

			point.action = visualization_msgs::Marker::ADD;

			points.markers.push_back(point);
		}
	}

	visPublisher.publish(points);
}

Eigen::Matrix<double, 4, 1> CostMapContainer::heightColor(double h){
	Eigen::Matrix<double, 4, 1> color;
	h = (h - floor(h))*6;

	double f = !((int)floor(h) & 1) ? (floor(h)-h) : (1+floor(h)-h);

	color(0) = 1.0;
	switch((int)floor(h)){
		case 0:
		color(1) = 1; color(2) = f; color(3) = 0;
		break;
		case 1:
		color(1) = f; color(2) = 1; color(3) = 0;
		break;
		case 2:
		color(1) = 0; color(2) = 1; color(3) = f;
		break;
		case 3:
		color(1) = 0; color(2) = f; color(3) = 1;
		break;
		case 4:
		color(1) = f; color(2) = 0; color(3) = 1;
		break;
		case 5:
		color(1) = 1; color(2) = 0; color(3) = f;
		break;
		default:
		color(1) = 1; color(2) = 0.5; color(3) = 0.5;
		break;
	}

	return color;
}

// Constructor
CostMap::CostMap(){
  ;
}

// Destructor
CostMap::~CostMap(){
	;
}

inline int CostMap::point2grid(double x, double y){
	return (int)((x - m_min_x)/m_res) + (int)((y - m_min_y)/m_res)*m_gridStep_y;
}

inline bool CostMap::isIntoMap(double x, double y){
	return (x >= m_min_x && y >= m_min_y && x < m_max_x && y < m_max_y);
}

void CostMap::swap(CostMap costmap){
	if(!checkSimilarity(costmap)){
		ROS_ERROR("[EXPLORER]: Cost Maps Cannot be Swapped");
		exit(-1);
	}

	for(int iy = 0; iy < m_gridSize_y; iy++){
		for(int ix = 0; ix < m_gridSize_x; ix++){
	  		int index = ix + iy*m_gridStep_y;
	  		m_grid[index] = costmap.getValueIdx(index);
		}
  	}
}

CostMap CostMap::copyTo(){
	CostMap costmap;

	costmap.m_gridSize_x = m_gridSize_x;
	costmap.m_gridSize_y = m_gridSize_y;
	costmap.m_gridSize = m_gridSize;
	costmap.m_gridStep_y = m_gridStep_y;
	costmap.m_max_x = m_max_x;
	costmap.m_max_y = m_max_y;
	costmap.m_max_z = m_max_z;
	costmap.m_min_x = m_min_x;
	costmap.m_min_y = m_min_y;
	costmap.m_min_z = m_min_z;
	costmap.cc_inc = cc_inc;
	costmap.cc_dec = cc_dec;
	costmap.m_res = m_res;
	costmap.rr = rr;
	costmap.m_grid = m_grid;
	
	return costmap;
}

double CostMap::evaluateCost(double x, double y){
	if(isIntoMap(x, y)){
		uint64_t ii = point2grid(x, y); 
		return m_grid[ii];
  	}

  return 0.0;
}

double CostMap::evaluateCostRadius(double x, double y){
	std::vector<int> eval_idx;
	for(double dx = x-rr; dx <= x+rr; dx += 0.1){
		for(double dy = y-rr; dy <= y+rr; dy += 0.1){
			double dr = sqrt(pow(x-dx, 2) + pow(y-dy, 2));
			if(dr > rr){
				continue;
			}

			uint64_t ii = point2grid(dx, dy); 
			if(ii >= m_grid.size() || ii < 0){
				continue;
			}

			if(eval_idx.size() == 0){
				eval_idx.push_back(ii);
			} else{
				bool found = false;
				for(std::vector<int>::iterator iter = eval_idx.begin();
						iter != eval_idx.end(); iter++){
					
					if(*iter == ii){
						found = true;
						break;
					}
				}

				if(!found){
					eval_idx.push_back(ii);
				}
			}
		}
	}

	double totalCost = 0.0;
	for(std::vector<int>::iterator iter = eval_idx.begin();
			iter != eval_idx.end(); iter++){

		totalCost += m_grid[*iter];
	}

	return totalCost;
}

void CostMap::updateCost(double x, double y, int action){
	if(isIntoMap(x, y)){
		uint64_t ii = point2grid(x, y);
		if(ii >= m_grid.size() || ii < 0){
			return;
		}

		double cc = m_grid[ii];

		if(action & INCREASE){
			cc = cc*cc_inc;
			cc = cc > 1 ? 1:cc;
		} else if(action & DECREASE){
			cc = cc*cc_dec;
			cc = cc < 0 ? 0:cc;
		} else{
			ROS_ERROR("[EXPLORER]: Action Not Implemented");
		}

		m_grid[ii] = cc;
  	}
}

void CostMap::updateCostRadius(double x, double y, int action){
	std::vector<int> upg_idx;
	for(double dx = x-rr; dx <= x+rr; dx += 0.1){
		for(double dy = y-rr; dy <= y+rr; dy += 0.1){
			double dr = sqrt(pow(x-dx, 2) + pow(y-dy, 2));
			if(dr > rr){
				continue;
			}

			uint64_t ii = point2grid(dx, dy); 
			if(ii >= m_grid.size() || ii < 0){
				continue;
			}

			if(upg_idx.size() == 0){
				upg_idx.push_back(ii);
			} else{
				bool found = false;
				for(std::vector<int>::iterator iter = upg_idx.begin();
					iter != upg_idx.end(); iter++){
					
					if(*iter == ii){
						found = true;
						break;
					}
				}

				if(!found){
					upg_idx.push_back(ii);
				}
			}
		}
	}

	for(std::vector<int>::iterator iter = upg_idx.begin();
		iter != upg_idx.end(); iter++){

		double cc = m_grid[*iter];

		if(action & INCREASE){
			cc = cc*cc_inc;
			cc = cc > 1 ? 1:cc;
		} else if(action & DECREASE){
			cc = cc*cc_dec;
			cc = cc < 0 ? 0:cc;
		} else{
			ROS_ERROR("[EXPLORER]: Action Not Implemented");
		}

		m_grid[*iter] = cc;
	}
}

void CostMap::updateAllCostMap(int action){
	for(int iy = 0; iy < m_gridSize_y; iy++){
		for(int ix = 0; ix < m_gridSize_x; ix++){
			int index = ix + iy*m_gridStep_y;
			double cc = m_grid[index];

			if(action & INCREASE){
				cc = cc*cc_inc;
				cc = cc > 1 ? 1:cc;
			} else if(action & DECREASE){
				cc = cc*cc_dec;
				cc = cc < 0 ? 0:cc;
			} else{
				ROS_ERROR("[EXPLORER]: Action Not Implemented");
			}

			m_grid[index] = cc;
		}
  	}
}


void CostMap::updateCostFromPoints(std::vector<Eigen::Matrix<double, 2, 1>> ps, int action){
	std::vector<int> upg_idx;
	for(std::vector<Eigen::Matrix<double, 2, 1>>::iterator pp_iter = ps.begin();
		pp_iter != ps.end(); pp_iter++){
		
		double x = (*pp_iter)(0);
		double y = (*pp_iter)(1);
		for(double dx = x-rr; dx <= x+rr; dx += 0.1){
			for(double dy = y-rr; dy <= y+rr; dy += 0.1){
				double dr = sqrt(pow(x-dx, 2) + pow(y-dy, 2));
				if(dr > rr){
					continue;
				}

				uint64_t ii = point2grid(dx, dy); 
				if(ii >= m_grid.size() || ii < 0){
					continue;
				}

				if(upg_idx.size() == 0){
					upg_idx.push_back(ii);
				} else{
					bool found = false;
					for(std::vector<int>::iterator iter = upg_idx.begin();
							iter != upg_idx.end(); iter++){
						
						if(*iter == ii){
							found = true;
							break;
						}
					}

					if(!found){
						upg_idx.push_back(ii);
					}
				}
			}
		}
	}

	for(std::vector<int>::iterator iter = upg_idx.begin();
		iter != upg_idx.end(); iter++){

		double cc = m_grid[*iter];

		if(action & INCREASE){
			cc = cc*cc_inc;
			cc = cc > 1 ? 1:cc;
		} else if(action & DECREASE){
			cc = cc*cc_dec;
			cc = cc < 0 ? 0:cc;
		} else{
			ROS_ERROR("[EXPLORER]: Action Not Implemented");
		}

		m_grid[*iter] = cc;
	}
}

double CostMap::getValueIdx(int idx){
	return m_grid[idx];
}

bool CostMap::checkSimilarity(CostMap costmap){
	if(m_gridSize_x != costmap.getGridSize_x()){
		return false;
	}

	if(m_gridSize_y != costmap.getGridSize_y()){
		return false;
	}

	return true;
}

int CostMap::getGridSize_x(){
	return m_gridSize_x;
}

int CostMap::getGridSize_y(){
	return m_gridSize_y;
}
