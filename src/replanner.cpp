#include <mav_planning/replanner.hpp>

using namespace _Replanner_;

// Constructor
Replanner::Replanner(ros::NodeHandle& nh, Environment* _env): environment(_env){
	nh.param("safe_margin", safe_margin, 0.5);
	nh.param("obstacle_size", obs_size, 4.0); // 1.0
	nh.param("max_ahead_time", dt_max, 2.0);
	nh.param("min_time_emergency", dt_min, 0.5);
	nh.param("check_time_resolution", dt_res, 0.1);
	nh.param("max_sample_pts", max_sample_pts, 3000);
	nh.param("max_sample_time", max_sample_time, 0.1);
	nh.param("x_inflate", si(0), 1.0);
	nh.param("y_inflate", si(1), 3.0);
	nh.param("z_inflate", si(2), 1.0);
	nh.param("map_resolution", m_res, 0.2);
	nh.param("maximum_paths_number", max_pts_nn, 10);
	nh.param("maximum_optimizing_paths_number", max_opt_paths_nn, 5);

	nh.param("max_x", m_max_x, 0.0);
	nh.param("max_y", m_max_y, 10.0);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, 0.0);
	nh.param("min_z", m_min_z, 0.0);

	nh.param("publish_debug_rep", debug_flag, true);
	nh.param("use_multi_threads", multi_thrs, true);
	nh.param("do_final_optimization", do_final_opt, false);

	spline = new BSpline<Eigen::Matrix<double, 3, 1>>(7);

	optimizer.resize(max_opt_paths_nn);
	for(uint ii = 0; ii < max_opt_paths_nn; ii++){
		optimizer[ii] = new Optimizer(nh, environment);
	}

	// std::srand(std::time(nullptr));
}

// Destructor
Replanner::~Replanner(){
	delete spline;

	for(uint ii = 0; ii < max_opt_paths_nn; ii++){
		delete optimizer[ii];
	}
}

void Replanner::setInflation(Eigen::Matrix<double, 3, 1> ii){
	si = ii;
}

void Replanner::setTrajectory(std::vector<Eigen::Matrix<double, 3, 1>> cp, double tt){
	is_prev_collision = false;
	spline->setControl(cp, tt);
}

void Replanner::getTrajectory(std::vector<Eigen::Matrix<double, 3, 1>>& cp, double& tt){
	cp = spline->getControl();
	tt = spline->getTime();
}

void Replanner::getPaths(std::vector<std::vector<Eigen::Matrix<double, 3, 1>>>& pts){
	pts = paths;
}

void Replanner::getTrajectories(std::vector<std::vector<Eigen::Matrix<double, 3, 1>>>& _cps){
	_cps = cps;
}

int Replanner::superviseTrajectory(double tt){
	if(spline->getControl().size() == 0){
		ROS_ERROR("[REPLANNER]: Trajectory Not Set");
		return NO_COLLISION;
	}

	// Use Last Seen PC
	pc = environment->getLastCloud();
	if(pc.empty()){
		ROS_INFO_COND(debug_flag, "[REPLANNER]: Point Cloud Empty");
        return NO_COLLISION;
    }

	// Build KdTree
	kdtree.setInputCloud(pc.makeShared());

	// Look Ahead into Trajectory
	std::vector<int> idx;
    std::vector<float> dds;
    pcl::PointXYZ pp_pcl;
	Eigen::Matrix<double, 3, 1> pp;
    for(double dt = 0.0; dt <= dt_max; dt += dt_res){
		pp = spline->evaluateCurve(tt+dt, 0);

        pp_pcl.x = pp(0);
        pp_pcl.y = pp(1);
        pp_pcl.z = pp(2);
        kdtree.radiusSearch(pp_pcl, safe_margin, idx, dds);

        if(idx.size() > 2){
			if(!is_prev_collision){
				ROS_INFO_COND(debug_flag, "[REPLANNER]: Detected Possible Collision");
				is_prev_collision = true;
				return NO_COLLISION;
			}

			ROS_INFO_COND(debug_flag, "[REPLANNER]: Detected Collision");
			
			if(dt < dt_min){
				ROS_WARN_COND(debug_flag, "[REPLANNER]: Detected Imminent Collision");
				is_prev_collision = false;
				return EMERGENCY_COLLISION;
			}

			if(replanTrajectory(tt+(dt/2), tt+dt)){
				is_prev_collision = false;
				return COLLISION;
			} else{
				ROS_WARN_COND(debug_flag, "[REPLANNER]: Replanning Went Wrong");
				is_prev_collision = false;
				return EMERGENCY_COLLISION;
			}
		}
	}

	is_prev_collision = false;
	return NO_COLLISION;
}

bool Replanner::replanTrajectory(double ti, double tc){
	Eigen::Matrix<double, 3, 1> pi;
	Eigen::Matrix<double, 3, 1> pf;
	Eigen::Matrix<double, 3, 1> pc;

	pi = spline->evaluateCurve(ti, 0);
	pc = spline->evaluateCurve(tc, 0);

	double initial_ll = (pi-pc).norm();

	bool found_last_pt = false;
	double tf = 0.0;

	Eigen::Matrix<double, 3, 1> pf_p = pc;
	for(double tt = tc; tt <= spline->getTime(); tt += dt_res){
		pf = spline->evaluateCurve(tt, 0);
		initial_ll += (pf_p - pf).norm();
		pf_p = pf;

		if((pc-pf).norm() >= obs_size){
			found_last_pt = true;
			tf = tt;
			break;
		}
	}

	if(!found_last_pt){
		return false;
	}

	findTopologicalPaths(pi, pf);

	if(paths.size() == 0){
		ROS_WARN_COND(debug_flag, "[REPLANNER]: No Paths Found");
		return false;
	}

	// Optimize Paths
	int idx = std::min((int)paths.size(), max_opt_paths_nn);
	cps.resize(idx);

	if(multi_thrs){
		std::vector<std::thread> threads;
		for(int ii = 0; ii < idx; ii++){
        	threads.emplace_back(&Replanner::optimizePath, this, paths[ii], ti, tf, ii);
		}

      	for(int ii = 0; ii < idx; ii++){
			threads[ii].join();
		}
	} else{
		for(int ii = 0; ii < idx; ii++){
			optimizePath(paths[ii], ti, tf, ii);
		}
	}

	// Select Best Trajectory
	sortTrajectories();

	BSpline<Eigen::Matrix<double, 3, 1>> traj(7);
	traj.setControl(cps.front(), 1.0);

	double new_ll = 0.0;
	Eigen::Matrix<double, 3, 1> pp_a;
	Eigen::Matrix<double, 3, 1> pp_p = traj.evaluateCurve(0.0, 0);
	for(double ii = 0; ii <= 1.0; ii += 0.1){
		pp_a = traj.evaluateCurve(ii, 0);
		new_ll += (pp_p-pp_a).norm();
		pp_p = pp_a;
	}

	double dt = new_ll*(tf-ti)/initial_ll;
	spline->insertSpline(cps.front(), ti, tf, dt);

	if(do_final_opt){
		std::vector<Eigen::Matrix<double, 3, 1>> cp = spline->getControl();
		double tt = spline->getTime();
		optimizer[0]->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE);

		spline->clearAll();
		spline->setControl(cp, tt);
	}

	return true;
}

void Replanner::optimizePath(std::vector<Eigen::Matrix<double, 3, 1>> pt, double ti, double tf, int idx){
	BSpline<Eigen::Matrix<double, 3, 1>> curve(7);
	std::vector<Eigen::Matrix<double, 3, 1>> pps = discretizePath(pt, 4);

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> bb;
	std::vector<Eigen::Matrix<double, 3, 1>> vv;
	vv.push_back(spline->evaluateCurve(ti, 1));
	vv.push_back(spline->evaluateCurve(tf, 1));

	std::vector<Eigen::Matrix<double, 3, 1>> aa;
	aa.push_back(spline->evaluateCurve(ti, 2));
	aa.push_back(spline->evaluateCurve(tf, 2));

	std::vector<Eigen::Matrix<double, 3, 1>> jj;
	jj.push_back(spline->evaluateCurve(ti, 3));
	jj.push_back(spline->evaluateCurve(tf, 3));

	bb.push_back(vv);
	bb.push_back(aa);
	bb.push_back(jj);

	double tt = tf-ti;
	curve.setWaypoints(pps, bb, tt);

	std::vector<Eigen::Matrix<double, 3, 1>> cp = curve.getControl();
	optimizer[idx]->setGuidePoints(pps);

	// TODO
	Eigen::Matrix<double, 6, 1> cc;
	optimizer[idx]->getCosts(cc);

	cc(0) = 100.0;
	cc(4) = 100.0;
	optimizer[idx]->setCosts(cc);
	optimizer[idx]->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE | Optimizer::GUIDE);

	// TODO
	//optimizer[idx]->optimizeBSpline(cp, tt, Optimizer::GUIDE_PHASE);
	//optimizer[idx]->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE);

	cps[idx] = cp;
}

void Replanner::findTopologicalPaths(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf){
	createGraph(pi, pf);

	extractPaths();
	shortcutPaths();
	pruneEquivalent();
	sortPaths();

	for(uint ii = 0; ii < graph.size(); ii++){
		delete graph[ii];
	}

	graph.clear();
	graph.shrink_to_fit();
}

void Replanner::extractPaths(){
	paths.clear();
	paths.shrink_to_fit();

	std::vector<GraphNode*> visited;
	visited.push_back(graph.front());
	depthSearch(visited);
}

void Replanner::depthSearch(std::vector<GraphNode*>& visited){
	GraphNode* node = visited.back();
	for(std::vector<GraphNode*>::iterator iter = node->neighbors.begin();
		iter != node->neighbors.end(); iter++){

		if((*iter)->id == 1){
			std::vector<Eigen::Matrix<double, 3, 1>> pt;
			bool first_element = false;
			for(std::vector<GraphNode*>::iterator iter_v = visited.begin();
				iter_v != visited.end(); iter_v++){

				pt.push_back((*iter_v)->pp);
			}

			pt.push_back((*iter)->pp);
			paths.push_back(pt);
			continue;

		} else{
			bool already_expanded = false;
			for(std::vector<GraphNode*>::iterator iter_e = visited.begin();
				iter_e != visited.end(); iter_e++){
			
				if((*iter)->id == (*iter_e)->id){
					already_expanded = true;
					break;
				}
			}

			if(already_expanded){
				continue;
			}

			visited.push_back(*iter);
			depthSearch(visited);
			visited.pop_back();

			if(paths.size() >= max_pts_nn){
				return;
			}
		}
	}
}

void Replanner::createGraph(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf){
	GraphNode* sn = new GraphNode(pi, GraphNode::GUARD, 0);
	GraphNode* en = new GraphNode(pf, GraphNode::GUARD, 1);

	graph.push_back(sn);
	graph.push_back(en);
	
	double dt = 0.5*(pf - pi).norm();
	Eigen::Matrix<double, 3, 1> bb = si + Eigen::Matrix<double, 3, 1>(dt, 0, 0);
	Eigen::Matrix<double, 3, 1> tt = 0.5*(pi + pf);

	Eigen::Matrix<double, 3, 3> rr;
	Eigen::Matrix<double, 3, 1> dw(0.0, 0.0, -1.0);
	Eigen::Matrix<double, 3, 1> xtf = (pf - tt).normalized();
	Eigen::Matrix<double, 3, 1> ytf = xtf.cross(dw).normalized();
	Eigen::Matrix<double, 3, 1> ztf = xtf.cross(ytf);

	rr.col(0) = xtf;
	rr.col(1) = ytf;
	rr.col(2) = ztf;

	int id = 1;
	int nn = 0;
	double tot_tt = 0.0;
	ros::Time rt;
	while(nn < max_sample_pts && tot_tt < max_sample_time){
		nn++;
		rt = ros::Time::now();

		Eigen::Matrix<double, 3, 1> pp = samplePoint(bb, tt, rr);
		if(getDistance(pp(0), pp(1), pp(2)) < safe_margin){
			tot_tt += (ros::Time::now() - rt).toSec();
			continue;
		}

		// Search Visible Guards
		std::vector<GraphNode*> guards = findVisibleGuards(pp);
		if(guards.size() == 0){
			GraphNode* newGuard = new GraphNode(pp, GraphNode::GUARD, ++id);
			graph.push_back(newGuard);

		}else if(guards.size() == 2){
			if(!needConnection(guards[0], guards[1], pp)) {
				tot_tt += (ros::Time::now() - rt).toSec();
				continue;
			}

			GraphNode* newConnector = new GraphNode(pp, GraphNode::CONNECTOR, ++id);
			graph.push_back(newConnector);

			guards[0]->neighbors.push_back(newConnector);
			guards[1]->neighbors.push_back(newConnector);

			newConnector->neighbors.push_back(guards[0]);
			newConnector->neighbors.push_back(guards[1]);
		}

		tot_tt += (ros::Time::now() - rt).toSec();
	}

	pruneGraph();
}

std::vector<GraphNode*> Replanner::findVisibleGuards(Eigen::Matrix<double, 3, 1> pp){
	std::vector<GraphNode*> visibleGuards;
	for(std::vector<GraphNode*>::iterator iter = graph.begin(); iter != graph.end(); iter++){
		if((*iter)->type == GraphNode::CONNECTOR){
			continue;
		}

		if(isVisible(pp, (*iter)->pp, safe_margin)){
			visibleGuards.push_back(*iter);
			if(visibleGuards.size() == 2){
				break;
			}
		}
	}

	return visibleGuards;
}

bool Replanner::needConnection(GraphNode* ni, GraphNode* nf, Eigen::Matrix<double, 3, 1> pp){
	std::vector<Eigen::Matrix<double, 3, 1>> pt_1(3);
	std::vector<Eigen::Matrix<double, 3, 1>> pt_2(3);

	pt_1[0] = ni->pp;
	pt_1[2] = nf->pp;

	pt_2[0] = ni->pp;
	pt_2[2] = nf->pp;

	pt_1[1] = pp;

	for(uint ii = 0; ii < ni->neighbors.size(); ii++){
		for(uint jj = 0; jj < nf->neighbors.size(); jj++){
			if(ni->neighbors[ii]->id == nf->neighbors[jj]->id){
				pt_2[1] = ni->neighbors[ii]->pp;
				
				if(samePath(pt_1, pt_2)){
					if(pathLength(pt_1) < pathLength(pt_2)){
						ni->neighbors[ii]->pp;
					}

					return false;
				}
			}			
		}
	}

	return true;
}

double Replanner::pathLength(std::vector<Eigen::Matrix<double, 3, 1>> pt){
	double ll = 0.0;
	if(pt.size() < 2){
		return ll;
	}

	for(uint ii = 0; ii < pt.size()-1; ii++){
		ll += (pt[ii+1] - pt[ii]).norm();
	}

	return ll;
}

bool Replanner::samePath(std::vector<Eigen::Matrix<double, 3, 1>> pt_1, std::vector<Eigen::Matrix<double, 3, 1>> pt_2){
	std::vector<Eigen::Matrix<double, 3, 1>> pts_1 = discretizePath(pt_1);
	std::vector<Eigen::Matrix<double, 3, 1>> pts_2 = discretizePath(pt_2, ceil(pts_1.size()/(pt_2.size()-1)));

	for(uint ii = 0; ii < pts_1.size(); ii++){
		if(!isVisible(pts_1[ii], pts_2[ii], m_res)){
			return false;
		}
	}

	return true;
}

std::vector<Eigen::Matrix<double, 3, 1>> Replanner::discretizePath(std::vector<Eigen::Matrix<double, 3, 1>> pt, int nn){
	std::vector<Eigen::Matrix<double, 3, 1>> pts;
	Eigen::Matrix<double, 3, 1> pi;
	Eigen::Matrix<double, 3, 1> pf;

	for(uint ii = 0; ii < pt.size()-1; ii++){
		pi = pt[ii];
		pf = pt[ii+1];

		double rr = (pi-pf).norm();
		double dr = rr/nn;

		Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();
		for(int jj = 0; jj <= nn; jj++){
			Eigen::Matrix<double, 3, 1> pp = pi + dr*jj*vv;
			pts.push_back(pp);
		}
	}

	return pts;
}

std::vector<Eigen::Matrix<double, 3, 1>> Replanner::discretizePath(std::vector<Eigen::Matrix<double, 3, 1>> pt){
	std::vector<Eigen::Matrix<double, 3, 1>> pts;
	Eigen::Matrix<double, 3, 1> pi;
	Eigen::Matrix<double, 3, 1> pf;
	for(uint ii = 0; ii < pt.size()-1; ii++){
		pi = pt[ii];
		pf = pt[ii+1];

		double rr = (pi-pf).norm();
		int nn = ceil(rr/m_res);

		double dr = rr/nn;

		Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();
		for(int jj = 0; jj <= nn; jj++){
			Eigen::Matrix<double, 3, 1> pp = pi + dr*jj*vv;
			pts.push_back(pp);
		}
	}

	return pts;
}

void Replanner::pruneGraph(){
	if(graph.size() > 2){
		for(std::vector<GraphNode*>::iterator iter_1 = graph.begin();
			iter_1 != graph.end(); iter_1++){
			
			if((*iter_1)->id <= 1){
				continue;
			}

			if((*iter_1)->neighbors.size() <= 1){
				for(std::vector<GraphNode*>::iterator iter_2 = graph.begin();
					iter_2 != graph.end(); iter_2++){
					
					for(std::vector<GraphNode*>::iterator iter_3 = (*iter_2)->neighbors.begin();
						iter_3 != (*iter_2)->neighbors.end(); iter_3++){
						
						if((*iter_3)->id == (*iter_1)->id){
							(*iter_2)->neighbors.erase(iter_3);
							break;
						}
					}
				}

				graph.erase(iter_1);
				iter_1 = graph.begin();
			}
		}
	}
}

void Replanner::shortcutPaths(){
	if(multi_thrs){
		std::vector<std::thread> threads;
		for(uint ii = 0; ii < paths.size(); ii++){
			threads.push_back(std::thread(&Replanner::shortcutPath, this, paths[ii], ii));
		}

		for(uint ii = 0; ii < paths.size(); ii++){
			threads[ii].join();
		}
	} else{
		for(uint ii = 0; ii < paths.size(); ii++){
			shortcutPath(paths[ii], ii);
		}
	}
}

void Replanner::shortcutPath(std::vector<Eigen::Matrix<double, 3, 1>> pt, int idx){
	std::vector<Eigen::Matrix<double, 3, 1>> spt;
	std::vector<Eigen::Matrix<double, 3, 1>> dpt = discretizePath(pt);

	Eigen::Matrix<double, 3, 1> grad;
	Eigen::Matrix<double, 3, 1> cp;
	Eigen::Matrix<double, 3, 1> vv;
	Eigen::Matrix<double, 3, 1> dir;

	spt.push_back(dpt.front());
	for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = dpt.begin();
		iter != dpt.end(); iter++){
		
		if(isVisible(spt.back(), *iter, safe_margin)){
			continue;
		}

		spt.push_back(*(iter-1));
	}
	
	spt.push_back(dpt.back());
	if(pathLength(pt) > pathLength(spt)){
		paths[idx] = spt;
	}
}

void Replanner::pruneEquivalent(){
	if(paths.size() < 2){
		return;
	}

	std::vector<int> path_id;
	path_id.push_back(0);

	for(int ii = 1; ii < paths.size(); ii++){
		bool is_new = true;
    	for(int jj = 0; jj < path_id.size(); jj++){
			if(samePath(paths[ii], paths[path_id[jj]])){
				is_new = false;
				break;
			}
		}

		if(is_new){
      		path_id.push_back(ii);
    	}
	}
  

	std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> pruned_paths;
  	for(int ii = 0; ii < path_id.size(); ii++){
    	pruned_paths.push_back(paths[path_id[ii]]);
  	}

	paths.clear();
	paths.shrink_to_fit();
	paths = pruned_paths;
}

void Replanner::sortPaths(){
	std::sort(paths.begin(), paths.end(), Replanner::sortByLenght);
}

void Replanner::sortTrajectories(){
	std::sort(cps.begin(), cps.end(), Replanner::sortByCost);
}

bool Replanner::sortByLenght(std::vector<Eigen::Matrix<double, 3, 1>> pt1, std::vector<Eigen::Matrix<double, 3, 1>> pt2){
	double ll1 = 0.0;
	if(pt1.size() > 1){
		for(uint ii = 0; ii < pt1.size()-1; ii++){
			ll1 += (pt1[ii+1] - pt1[ii]).norm();
		}	
	}

	double ll2 = 0.0;
	if(pt2.size() > 1){
		for(uint ii = 0; ii < pt2.size()-1; ii++){
			ll2 += (pt2[ii+1] - pt2[ii]).norm();
		}	
	}

	return ll1 < ll2;
}

bool Replanner::sortByCost(std::vector<Eigen::Matrix<double, 3, 1>> pt1, std::vector<Eigen::Matrix<double, 3, 1>> pt2){
	BSpline<Eigen::Matrix<double, 3, 1>> sp1(7);
	BSpline<Eigen::Matrix<double, 3, 1>> sp2(7);

	sp1.setControl(pt1, 1.0);
	sp2.setControl(pt2, 1.0);

	Eigen::Matrix<double, 3, 1> tj1 = sp1.getTotalJerk();
	Eigen::Matrix<double, 3, 1> tj2 = sp2.getTotalJerk();

	return tj1.norm() < tj2.norm();
}

bool Replanner::isVisible(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf, double thr){
	double rr = (pi-pf).norm();
	Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();

	for(double dr = 0.0; dr < rr; dr += 0.1){
		Eigen::Matrix<double, 3, 1> pp = pi + vv*dr;
		if(getDistance(pp(0), pp(1), pp(2)) <= thr){
			return false;
		}
	}

	return true;
}

Eigen::Matrix<double, 3, 1> Replanner::samplePoint(Eigen::Matrix<double, 3, 1> bb,
												   Eigen::Matrix<double, 3, 1> tt,
												   Eigen::Matrix<double, 3, 3> rr){
	Eigen::Matrix<double, 3, 1> pp;
	do{
		pp(0) = (((double)std::rand()/(double)RAND_MAX)*2*bb(0)) - bb(0);
		pp(1) = (((double)std::rand()/(double)RAND_MAX)*2*bb(1)) - bb(1);
		pp(2) = (((double)std::rand()/(double)RAND_MAX)*2*bb(2)) - bb(2);
		pp = (rr*pp) + tt;
	} while(!insideBounds(pp));

	return pp;
}

bool Replanner::insideBounds(Eigen::Matrix<double, 3, 1> pp){
	if(pp(0) > m_max_x || pp(0) < m_min_x){
		return false;
	}
  	if(pp(1) > m_max_y || pp(1) < m_min_y){
		return false;
	}
	if(pp(2) > m_max_z || pp(2) < m_min_z){
		return false;
	}
  	return true;
}

double Replanner::getDistance(double x, double y, double z){
	std::vector<int> idx;
    std::vector<float> dds;
    pcl::PointXYZ pp;
	pp.x = x;
	pp.y = y;
	pp.z = y;

	if(pc.empty()){
		return environment->evaluateSDF(x, y, z);
	}

	if(kdtree.nearestKSearch(pp, 1, idx, dds) > 0){
		return std::min((double)sqrt(dds[0]), environment->evaluateSDF(x, y, z));
	} else{
		return environment->evaluateSDF(x, y, z);
	}
}