#include <mav_planning/inspector.hpp>

using namespace _Inspector_;

// Constructor
Inspector::Inspector(ros::NodeHandle& nh, Environment* _env): environment(_env){
	nh.param("delta_angle", delta_th, 0.5);
	nh.param("delta_zz", delta_zz, 0.45);
	nh.param("max_inspect_radius", max_rr, 1.7);
	nh.param("opt_inspect_radius", opt_rr, 1.1);
	nh.param("safe_margin", safe_margin, 0.6);
	nh.param("safety_relaxing_factor", ff_relax, 0.1);
	nh.param("nn_sampling_points", nn_max, 700);
	nh.param("max_iterations_tsp", max_iter, 20000);
	nh.param("update_lr", up_lr, 0.99997);
	nh.param("initial_lr", init_lr, 0.8);
	nh.param("update_nn", up_nn, 0.9997);
	nh.param("x_inflate_inps", si(0), 0.5);
	nh.param("y_inflate_inps", si(1), 0.5);
	nh.param("z_inflate_inps", si(2), 2.0);
	nh.param("max_sampling_time", max_tt, 0.02);
	nh.param("max_paths_nn", max_nn_pts, 5);
	nh.param("max_inspection_vel", max_vv, 0.2);
	nh.param("new_smooth_cost", smooth_cc, 100.0);
	nh.param("new_guide_cost", guide_cc, 100.0);

	nh.param("publish_debug_ins", debug_flag, true);
	nh.param("use_last_point", use_last_pp, false);
	
	nh.param("max_x", m_max_x, 0.0);
	nh.param("max_y", m_max_y, 10.0);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, 0.0);
	nh.param("min_z", m_min_z, 0.0);
	nh.param("map_resolution", mm_res, 0.2);

	optimizer = new Optimizer(nh, environment);

	Eigen::Matrix<double, 6, 1> cc;
	optimizer->getCosts(cc);

	cc(0) = smooth_cc;
	cc(4) = guide_cc;
	optimizer->setCosts(cc);
	optimizer->setVelocityBound(max_vv);

	// std::srand(std::time(nullptr));
}

// Destructor
Inspector::~Inspector(){
	delete optimizer;
}

void Inspector::clearAll(){
	id = 0;

	vp_graph.clear();
	vp_graph.shrink_to_fit();

	ll_graph.clear();
	ll_graph.shrink_to_fit();

	pth.clear();
	pth.shrink_to_fit();

	cp.clear();
	cp.shrink_to_fit();

	cpy.clear();
	cpy.shrink_to_fit();

	paths.clear();
	paths.shrink_to_fit();
}

void Inspector::inspect(Eigen::Matrix<double, 2, 1> pp){
	clearAll();

	ros::Time t1 = ros::Time::now();
	generateViewpoints(pp);

	if(vp_graph.size() == 0){
		return;
	}

	ros::Time t2 = ros::Time::now();
	solveTSP();
	ros::Time t3 = ros::Time::now();
	correctPath();
	ros::Time t4 = ros::Time::now();
	computeTrajectory(pp);
	ros::Time t5 = ros::Time::now();

	ROS_INFO_COND(debug_flag, "[INSPECTOR]: Elapesed Times VP: %f TSP: %f CP: %f CT: %f",
				 (t2-t1).toSec(), (t3-t2).toSec(), (t4-t3).toSec(), (t5-t4).toSec());
}

void Inspector::computeTrajectory(Eigen::Matrix<double, 2, 1> pp){
	std::vector<Eigen::Matrix<double, 3, 1>> pps = discretizePath_dd(pth, mm_res*2);
	tt = pathLength(pps)/max_vv;

	cp.insert(cp.end(), 3, pps.front());
	cp.insert(cp.end(), pps.begin(), pps.end());
	cp.insert(cp.end(), 3, pps.back());

	optimizer->setGuidePoints(pps);
	optimizer->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE | Optimizer::GUIDE);

	// Generate Yaw CP
	BSpline<double> curve_y(3);
	std::vector<double> wp_y;
	std::vector<double> vv;
	std::vector<std::vector<double>> bb;

	vv.insert(vv.begin(), 2, 0.0);
	bb.push_back(vv);

	wp_y.push_back(to_2PI(atan2(dy(pps.front(), pp), dx(pps.front(), pp))));
	for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = pps.begin()+1;
		iter != pps.end(); iter++){

		double yy = to_2PI(atan2(dy(*iter, pp), dx(*iter, pp)));
		double prev_yy = to_2PI(wp_y.back());

		double dy = yy - prev_yy;
		double dy_2PI = abs(dy) - 2*M_PI;

		if(dy < 0){
			dy_2PI = -dy_2PI;
		}

		dy = abs(dy) < abs(dy_2PI) ? dy : dy_2PI;
		wp_y.push_back(wp_y.back() + dy);
	}

	curve_y.setWaypoints(wp_y, bb, tt);
	cpy = curve_y.getControl();
}

void Inspector::generateViewpoints(Eigen::Matrix<double, 2, 1> pp){
	Eigen::Matrix<double, 3, 1> pa;
	for(double zz = safe_margin+m_min_z; zz < m_max_z-mm_res; zz += delta_zz){
		pa(0) = pp(0);
		pa(1) = pp(1);
		pa(2) = zz;

		std::vector<Eigen::Matrix<double, 3, 1>> pps;
		bool continue_sampling = generateLayerPoints(pps, pa);

		if(pps.size() != 0){
			for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = pps.begin();
				iter != pps.end(); iter++){

				Node node;
				node.pp = (*iter);
				node.id = id++;
				node.cc = pa;

				vp_graph.push_back(node);
			}
		}

		if(!continue_sampling){
			break;
		}
	}
}

bool Inspector::generateLayerPoints(std::vector<Eigen::Matrix<double, 3, 1>>& pps, Eigen::Matrix<double, 3, 1> pp){
	bool find_notVisible = false;
	for(double th = 0.0; th < 2*M_PI; th += delta_th){
		std::vector<Eigen::Matrix<double, 3, 1>> all_pps;

		// Sample nn Points
		for(int ii = 0; ii < nn_max; ii++){
			Eigen::Matrix<double, 3, 1> pi = samplePoint(th, th+delta_th, pp);

			if(isLineVisible(pi, pp, 0.5)){//mm_res*2)){
				continue;
			}

			find_notVisible = true;
			if(isFeasible(pi)){
				all_pps.push_back(pi);
			}
		}

		if(all_pps.size() == 0){
			continue;
		}

		Eigen::Matrix<double, 3, 1> accumulator = Eigen::MatrixXd::Zero(3,1);
		double ww_accumulator = 0.0;
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = all_pps.begin();
			iter != all_pps.end(); iter++){

			double dd = pow(((*iter)-pp).norm() - opt_rr, 2);
			if(dd < 1e-1){
				dd = 1e-1;
			}

			double ww = 1/dd;
			accumulator += ww*(*iter);
			ww_accumulator += ww;
		}

		Eigen::Matrix<double, 3, 1> pp_nn = accumulator/ww_accumulator;
		if(isFeasible(pp_nn)){
			pps.push_back(pp_nn);
		}
	}

	return find_notVisible;
}

void Inspector::solveTSP(){
	Eigen::Matrix<double, 3, 1> pp;
	std::vector<Eigen::Matrix<double, 3, 1>> net;
	for(int ii = 0; ii < vp_graph.size()*8; ii++){
		pp(0) = ((double)std::rand()/(double)RAND_MAX);
		pp(1) = ((double)std::rand()/(double)RAND_MAX);
		pp(2) = ((double)std::rand()/(double)RAND_MAX);

		net.push_back(pp);
	}

	double nn = (double)net.size();
	double lr = init_lr;

	for(int ii = 0; ii < max_iter; ii++){
		int rr_idx = std::rand() % vp_graph.size();

		int idx_ = 0;
		int opt_idx = 0;
		double opt_dd = INFINITY;
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = net.begin();
			iter != net.end(); iter++){

			double dd = (vp_graph[rr_idx].pp - *iter).squaredNorm();
			if(dd < opt_dd){
				opt_dd = dd;
				opt_idx = idx_;
			}

			idx_++;
		}

		for(int ii = 0; ii < net.size(); ii++){
			int delta = abs(opt_idx - ii);
			int dist = std::min((int)delta, (int)net.size()-delta);
			int rr = floor(nn/10);

			rr = rr < 1 ? 1:rr;

			double gain = exp(-pow((double)dist,2)/(2*pow((double)rr,2)));
			net[ii] += gain*lr*(vp_graph[rr_idx].pp - net[ii]);
		}

		// Update Variables
		lr = lr*up_lr;
		nn = nn*up_nn;

		// Check Terminal Conditions
		if(lr < 0.001 || nn < 1){
			break;
		}
	}

	// Extract Result
	std::vector<Node> result;
	for(std::vector<Node>::iterator iter = vp_graph.begin();
		iter != vp_graph.end(); iter++){
		
		int idx_ = 0;
		int opt_idx = 0;
		double opt_dd = INFINITY;
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter_ = net.begin();
			iter_ != net.end(); iter_++){

			double dd = (iter->pp - *iter_).squaredNorm();
			if(dd < opt_dd){
				opt_dd = dd;
				opt_idx = idx_;
			}

			idx_++;
		}

		Node node;
		node.pp = iter->pp;
		node.id = opt_idx;
		result.push_back(node);
	}

	// Reorder Result
	std::sort(result.begin(), result.end(), sortResult);

	double max_dd = 0;
	std::vector<Node>::iterator ii_iter_opt;
	std::vector<Node>::iterator ff_iter_opt;
	for(std::vector<Node>::iterator iter = result.begin();
		iter != result.end(); iter++){
		
		std::vector<Node>::iterator ii_iter = iter;
		std::vector<Node>::iterator ff_iter = iter+1;

		if(ff_iter == result.end()){
			ff_iter = result.begin();
		}

		double dd = (ii_iter->pp - ff_iter->pp).squaredNorm();
		if(dd > max_dd){
			ii_iter_opt = ii_iter;
			ff_iter_opt = ff_iter;
			max_dd = dd;
		}
	}

	while(ff_iter_opt != ii_iter_opt){
		pth.push_back(ff_iter_opt->pp);

		ff_iter_opt++;
		if(ff_iter_opt == result.end()){
			ff_iter_opt = result.begin();
		}
	}

	pth.push_back(ff_iter_opt->pp);
}

void Inspector::correctPath(){
	std::vector<Eigen::Matrix<double, 3, 1>> nn_pth;
	nn_pth.push_back(pth.front());

	if(use_last_pp){
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = pth.begin();
			iter != pth.end(); iter++){

			std::vector<Eigen::Matrix<double, 3, 1>>::iterator ii_iter = iter;
			std::vector<Eigen::Matrix<double, 3, 1>>::iterator ff_iter = iter+1;

			if(ff_iter == pth.end()){
				ff_iter = pth.begin();
			}

			if(isLineVisible(*ii_iter, *ff_iter, safe_margin)){
				nn_pth.push_back(*ff_iter);
			} else{
				double elapsed_tt = 0;
				computePath(*ii_iter, *ff_iter);

				if(paths.size() == 0){
					ROS_ERROR("[INSPECTOR]: Fatal Error!");
					exit(-1);
				}

				for(uint ii = 1; ii < paths[0].size(); ii++){
					nn_pth.push_back(paths[0][ii]);
				}
			}
		}
	} else{
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = pth.begin();
			iter != pth.end()-1; iter++){

			std::vector<Eigen::Matrix<double, 3, 1>>::iterator ii_iter = iter;
			std::vector<Eigen::Matrix<double, 3, 1>>::iterator ff_iter = iter+1;

			if(isLineVisible(*ii_iter, *ff_iter, safe_margin)){
				nn_pth.push_back(*ff_iter);
			} else{
				double elapsed_tt = 0;
				computePath(*ii_iter, *ff_iter);

				if(paths.size() == 0){
					ROS_ERROR("[INSPECTOR]: Fatal Error!");
					exit(-1);
				}

				for(uint ii = 1; ii < paths[0].size(); ii++){
					nn_pth.push_back(paths[0][ii]);
				}
			}
		}
	}

	pth = nn_pth;
}

void Inspector::computePath(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf){
	paths.clear();
	paths.shrink_to_fit();

	ll_graph.clear();
	ll_graph.shrink_to_fit();

	GraphNode* sn = new GraphNode(pi, GraphNode::GUARD, 0);
	GraphNode* en = new GraphNode(pf, GraphNode::GUARD, 1);

	ll_graph.push_back(sn);
	ll_graph.push_back(en);
	
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
	double tot_tt = 0.0;

	ros::Time rt;
	while(tot_tt < max_tt){
		rt = ros::Time::now();

		Eigen::Matrix<double, 3, 1> pp = samplePoint(bb, tt, rr);
		if(environment->evaluateSDF(pp(0), pp(1), pp(2)) < safe_margin-ff_relax){
			tot_tt += (ros::Time::now() - rt).toSec();
			continue;
		}

		// Search Visible Guards
		std::vector<GraphNode*> guards = findVisibleGuards(pp);
		if(guards.size() == 0){
			GraphNode* newGuard = new GraphNode(pp, GraphNode::GUARD, ++id);
			ll_graph.push_back(newGuard);

		}else if(guards.size() == 2){
			if(!needConnection(guards[0], guards[1], pp)) {
				tot_tt += (ros::Time::now() - rt).toSec();
				continue;
			}

			GraphNode* newConnector = new GraphNode(pp, GraphNode::CONNECTOR, ++id);
			ll_graph.push_back(newConnector);

			guards[0]->neighbors.push_back(newConnector);
			guards[1]->neighbors.push_back(newConnector);

			newConnector->neighbors.push_back(guards[0]);
			newConnector->neighbors.push_back(guards[1]);
		}

		tot_tt += (ros::Time::now() - rt).toSec();
	}

	pruneGraph();
	extractPaths();
	shortcutPaths();

	std::sort(paths.begin(), paths.end(), Inspector::sortByLenght);
}

void Inspector::pruneGraph(){
	if(ll_graph.size() > 2){
		for(std::vector<GraphNode*>::iterator iter_1 = ll_graph.begin();
			iter_1 != ll_graph.end(); iter_1++){
			
			if((*iter_1)->id <= 1){
				continue;
			}

			if((*iter_1)->neighbors.size() <= 1){
				for(std::vector<GraphNode*>::iterator iter_2 = ll_graph.begin();
					iter_2 != ll_graph.end(); iter_2++){
					
					for(std::vector<GraphNode*>::iterator iter_3 = (*iter_2)->neighbors.begin();
						iter_3 != (*iter_2)->neighbors.end(); iter_3++){
						
						if((*iter_3)->id == (*iter_1)->id){
							(*iter_2)->neighbors.erase(iter_3);
							break;
						}
					}
				}

				ll_graph.erase(iter_1);
				iter_1 = ll_graph.begin();
			}
		}
	}
}

std::vector<GraphNode*> Inspector::findVisibleGuards(Eigen::Matrix<double, 3, 1> pp){
	std::vector<GraphNode*> visibleGuards;
	for(std::vector<GraphNode*>::iterator iter = ll_graph.begin(); iter != ll_graph.end(); iter++){
		if((*iter)->type == GraphNode::CONNECTOR){
			continue;
		}

		if(isLineVisible(pp, (*iter)->pp, safe_margin-ff_relax)){
			visibleGuards.push_back(*iter);
			if(visibleGuards.size() == 2){
				break;
			}
		}
	}

	return visibleGuards;
}

bool Inspector::needConnection(GraphNode* ni, GraphNode* nf, Eigen::Matrix<double, 3, 1> pp){
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

void Inspector::extractPaths(){
	paths.clear();
	paths.shrink_to_fit();

	std::vector<GraphNode*> visited;
	visited.push_back(ll_graph.front());
	depthSearch(visited);
}

void Inspector::depthSearch(std::vector<GraphNode*>& visited){
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

			if(paths.size() >= max_nn_pts){
				return;
			}
		}
	}
}

void Inspector::shortcutPaths(){
	std::vector<std::thread> threads;
	for(uint ii = 0; ii < paths.size(); ii++){
		threads.push_back(std::thread(&Inspector::shortcutPath, this, paths[ii], ii));
	}

	for(uint ii = 0; ii < paths.size(); ii++){
		threads[ii].join();
	}
}

void Inspector::shortcutPath(std::vector<Eigen::Matrix<double, 3, 1>> pt, int idx){
	std::vector<Eigen::Matrix<double, 3, 1>> spt;
	std::vector<Eigen::Matrix<double, 3, 1>> dpt = discretizePath_dd(pt, mm_res);

	Eigen::Matrix<double, 3, 1> grad;
	Eigen::Matrix<double, 3, 1> cp;
	Eigen::Matrix<double, 3, 1> vv;
	Eigen::Matrix<double, 3, 1> dir;

	spt.push_back(dpt.front());
	for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = dpt.begin();
		iter != dpt.end(); iter++){
		
		if(isLineVisible(spt.back(), *iter, safe_margin-ff_relax)){
			continue;
		}

		spt.push_back(*(iter-1));
	}
	
	spt.push_back(dpt.back());
	if(pathLength(pt) > pathLength(spt)){
		paths[idx] = spt;
	}
}

bool Inspector::samePath(std::vector<Eigen::Matrix<double, 3, 1>> pt_1, std::vector<Eigen::Matrix<double, 3, 1>> pt_2){
	std::vector<Eigen::Matrix<double, 3, 1>> pts_1 = discretizePath_dd(pt_1, mm_res);
	std::vector<Eigen::Matrix<double, 3, 1>> pts_2 = discretizePath_nn(pt_2, ceil(pts_1.size()/(pt_2.size()-1)));

	for(uint ii = 0; ii < pts_1.size(); ii++){
		if(!isLineVisible(pts_1[ii], pts_2[ii], mm_res)){
			return false;
		}
	}

	return true;
}

double Inspector::pathLength(std::vector<Eigen::Matrix<double, 3, 1>> pt){
	double ll = 0.0;
	if(pt.size() < 2){
		return ll;
	}

	for(uint ii = 0; ii < pt.size()-1; ii++){
		ll += (pt[ii+1] - pt[ii]).norm();
	}

	return ll;
}

bool Inspector::sortByLenght(std::vector<Eigen::Matrix<double, 3, 1>> pt1, std::vector<Eigen::Matrix<double, 3, 1>> pt2){
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

bool Inspector::isFeasible(Eigen::Matrix<double, 3, 1> pp){
	// Check if Safe Distance
	if(environment->evaluateSDF(pp(0), pp(1), pp(2)) < safe_margin){
		return false;
	}

	// Check if Inside Obstacle
	Eigen::Matrix<double, 3, 1> pz = pp;
	pz(2) = m_max_z-mm_res;

	if(!isLineVisible(pp, pz, mm_res)){
		return false;
	}

	return true;
}

Eigen::Matrix<double, 3, 1> Inspector::samplePoint(double th_mn, double th_mx, Eigen::Matrix<double, 3, 1> pa){
	Eigen::Matrix<double, 3, 1> pp;
	double th_dd = (th_mx - th_mn)/2;
	double th_cc = th_mn + th_dd;

	do{
		double th = th_cc + (((double)std::rand()/(double)RAND_MAX)*(2*th_dd) - th_dd);
		double rr = ((double)std::rand()/(double)RAND_MAX)*max_rr;

		pp(0) = pa(0) + cos(th)*rr;
		pp(1) = pa(1) + sin(th)*rr;
		pp(2) = pa(2);

	} while(!isInsideBounds(pp));

	return pp;
}

Eigen::Matrix<double, 3, 1> Inspector::samplePoint(Eigen::Matrix<double, 3, 1> bb,
												   Eigen::Matrix<double, 3, 1> tt,
												   Eigen::Matrix<double, 3, 3> rr){
	Eigen::Matrix<double, 3, 1> pp;
	do{
		pp(0) = (((double)std::rand()/(double)RAND_MAX)*2*bb(0)) - bb(0);
		pp(1) = (((double)std::rand()/(double)RAND_MAX)*2*bb(1)) - bb(1);
		pp(2) = (((double)std::rand()/(double)RAND_MAX)*2*bb(2)) - bb(2);
		pp = (rr*pp) + tt;
	} while(!isInsideBounds(pp));

	return pp;
}

bool Inspector::isLineVisible(Eigen::Matrix<double, 3, 1> pi, Eigen::Matrix<double, 3, 1> pf, double thr){
	double rr = (pi-pf).norm();
	Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();

	if(rr < mm_res){
		return true;
	}

	for(double dr = 0.0; dr < rr; dr += mm_res){
		Eigen::Matrix<double, 3, 1> pp = pi + vv*dr;
		if(environment->evaluateSDF(pp(0), pp(1), pp(2)) < thr){
			return false;
		}
	}

	return true;
}

std::vector<Eigen::Matrix<double, 3, 1>> Inspector::discretizePath_dd(std::vector<Eigen::Matrix<double, 3, 1>> pt, double dd){
	std::vector<Eigen::Matrix<double, 3, 1>> pts;
	Eigen::Matrix<double, 3, 1> pi;
	Eigen::Matrix<double, 3, 1> pf;
	for(uint ii = 0; ii < pt.size()-1; ii++){
		pi = pt[ii];
		pf = pt[ii+1];

		double rr = (pi-pf).norm();
		int nn = ceil(rr/dd);

		double dr = rr/nn;

		Eigen::Matrix<double, 3, 1> vv = (pf-pi).normalized();
		for(int jj = 0; jj <= nn; jj++){
			Eigen::Matrix<double, 3, 1> pp = pi + dr*jj*vv;
			pts.push_back(pp);
		}
	}

	return pts;
}

std::vector<Eigen::Matrix<double, 3, 1>> Inspector::discretizePath_nn(std::vector<Eigen::Matrix<double, 3, 1>> pt, int nn){
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

bool Inspector::isInsideBounds(Eigen::Matrix<double, 3, 1> pp){
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

void Inspector::getInspectPoints(std::vector<Eigen::Matrix<double, 3, 1>>& pps){
	for(int ii = 0; ii < vp_graph.size(); ii++){
		pps.push_back(vp_graph[ii].pp);
	}
}

void Inspector::getPath(std::vector<Eigen::Matrix<double, 3, 1>>& pps){
	pps = pth;
}

void Inspector::getTrajectory(std::vector<Eigen::Matrix<double, 3, 1>>& cp_, std::vector<double>& cpy_, double& tt_){
	cp_ = cp;
	cpy_ = cpy;
	tt_ = tt;
}