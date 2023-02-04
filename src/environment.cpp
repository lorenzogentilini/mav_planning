#include <mav_planning/environment.hpp>

// Environment Functions /////////////////////////////////////////////////////////
Environment::Environment(ros::NodeHandle& nh):  _tfListener(_tfBuffer),
	octreePublisher(nh.advertise<octomap_msgs::Octomap>("/octomap", 1)),
	binaryPublisher(nh.advertise<octomap_msgs::Octomap>("/octomap_binary", 1)),
	visualPublisher(nh.advertise<sensor_msgs::PointCloud2> ("/occupancy_map", 1)),
	debugPublisher(nh.advertise<sensor_msgs::PointCloud2> ("/mav_planning_debug", 1)),
	updateOctomap_timer(nh.createTimer(ros::Duration(0.1), &Environment::updateOctomap, this)),
	updateSdf_timer(nh.createTimer(ros::Duration(0.25), &Environment::updateSDF, this)),
	saveServer(nh.advertiseService("/save_map", &Environment::saveMap, this)),
	#ifdef MAVROS
	depthSubscriber(nh, "/d400/depth/color/points", 1),
	odomSubscriber(nh, "/mavros/local_position/odom", 1),
	#endif
	#ifdef FLIGHTMARE
	depthSubscriber(nh, "/cloud_in", 1),
	odomSubscriber(nh, "/odom", 1),
	#endif
	sync(syncPolicy(10), depthSubscriber, odomSubscriber){

	nh.param("px_sensor", ps_x, 0.05);
	nh.param("py_sensor", ps_y, 0.1);
	nh.param("pz_sensor", ps_z, 0.1);
	nh.param("qx_sensor", qs_x, -0.5);
	nh.param("qy_sensor", qs_y, 0.5);
	nh.param("qz_sensor", qs_z, -0.5);
	nh.param("qw_sensor", qs_w, 0.5);

	nh.param("min_sensor_r", min_r, 0.3);
	nh.param("max_sensor_r", max_r, 5.0);
	nh.param("max_cutoff_dd", maximum_dd, 20.0);

	nh.param("max_x", m_max_x, 0.0);
	nh.param("max_y", m_max_y, 10.0);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, 0.0);
	nh.param("min_z", m_min_z, 0.0);
	nh.param("color_factor", color_factor, 0.8);

	nh.param("map_name", mapName, std::string("/home/gent/catkin_ws/src/flightmare/flightrender/pointcloud_data/leonardo.ply"));

	nh.param("map_resolution", m_res, 0.15);
	nh.param("prob_hitting", p_hitting, 0.7);
	nh.param("prob_missing", p_missing, 0.4);
	nh.param("min_thr", min_thr, 0.12);
	nh.param("max_thr", max_thr, 0.97);
	nh.param("occ_thr", occ_thr, 0.5);
	nh.param("insertion_margin", insertion_margin, 0.6);
	nh.param("use_previous_map", use_map, true);
	nh.param("publish_debug_env", debug_flag, true);
	nh.param("downsample_resolution", ds_r, 0.1);
	nh.param("mean_kk_neighbors", mean_kkn, 30);
	nh.param("std_deviation", std_dev, 1.0);

	if(use_map){
		pcl::io::loadPLYFile(mapName, pc_map);
		kdtree.setInputCloud(pc_map.makeShared());
	}

	sync.registerCallback(boost::bind(&Environment::cloudCallback, this, _1, _2));
	
	octree = new octomap::OcTree(m_res);
	octree->setProbHit(p_hitting);
	octree->setProbMiss(p_missing);
	octree->setClampingThresMin(min_thr);
	octree->setClampingThresMax(max_thr);
	octree->setOccupancyThres(occ_thr);

	sdf_map = new SDFMap(nh);

	sensor_toBody << 1-2*pow(qs_y, 2)-2*pow(qs_z, 2), 2*qs_x*qs_y-2*qs_w*qs_z, 2*qs_x*qs_z+2*qs_w*qs_y, ps_x,
					 2*qs_x*qs_y+2*qs_w*qs_z, 1-2*pow(qs_x, 2)-2*pow(qs_z, 2), 2*qs_y*qs_z-2*qs_w*qs_x, ps_y,
					 2*qs_x*qs_z-2*qs_w*qs_y, 2*qs_y*qs_z+2*qs_w*qs_x, 1-2*pow(qs_x, 2)-2*pow(qs_y, 2), ps_z,
					 0, 0, 0, 1;

	#ifdef FLIGHTMARE
	flightmare_rotation << cos(M_PI/2), -sin(M_PI/2), 0, 0,
						   sin(M_PI/2), cos(M_PI/2), 0, 0,
						   0, 0, 1, 0,
						   0, 0, 0, 1;
	#endif
}

Environment::~Environment(){
	delete octree;
	delete sdf_map;
}

void Environment::updateSDF(const ros::TimerEvent& e){
	if(sdf_updating){
		return;
	}

	// Wait For Octomap Updation Completion
	while(octomap_updating){
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	sdf_updating = true;
	octomap_stopper = true;

	// Copy Current Vectors
	std::vector<Eigen::Matrix<double, 3, 1>> toBe_inserted_cc = toBe_inserted;
	std::vector<Eigen::Matrix<double, 3, 1>> toBe_removed_cc = toBe_removed;

	// Clear Vectors
	toBe_inserted.clear();
	toBe_inserted.shrink_to_fit();
	toBe_removed.clear();
	toBe_removed.shrink_to_fit();

	octomap_stopper = false;

	// Update SDF
	sdf_map->update(toBe_inserted_cc, toBe_removed_cc, octree);

	sdf_updating = false;
}

void Environment::updateOctomap(const ros::TimerEvent& e){
	if(octomap_updating || octomap_stopper || current_pc_.empty()){
		return;
	}

	octomap_updating = true;

	// Insert Point Cloud in Octomap
	octomap::point3d sensor_origin(sensor_origin_.x(), sensor_origin_.y(), sensor_origin_.z());
	pcl::PointCloud<pcl::PointXYZ> insert_cloud;

	pcl::PointCloud<pcl::PointXYZ> current_pc = pcl::PointCloud<pcl::PointXYZ>(current_pc_);
	pcl::PointCloud<pcl::PointXYZ> current_pc_full = pcl::PointCloud<pcl::PointXYZ>(current_pc_full_);

	// Check if a Point is far From Known Obstacles
	// if(use_map){
	// 	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = current_pc.begin(); iter != current_pc.end(); iter++){
	// 		std::vector<int> point_idx;
	// 		std::vector<float> sq_distance;
	// 		pcl::PointXYZ search_point = pcl::PointXYZ(iter->x, iter->y, iter->z);
	// 		kdtree.radiusSearch(search_point, insertion_margin, point_idx, sq_distance);

	// 		if(point_idx.size() != 0){
	// 			continue;
	// 		}

	// 		insert_cloud.push_back(*iter);
	// 	}
	// } else{
	// 	insert_cloud = current_pc;
	// }

	// Do Nothing
	insert_cloud = current_pc;

	if(insert_cloud.empty()){
		// No Points To Be Inserted
		octomap_updating = false;
		return;
	}

	bool occ_p, occ_a;

	// Update In a Sphere Around Actual Position
	double ddxy = 0.65;
	double ddz = 0.65;

	for(double xx = sensor_origin.x() - ddxy; xx <= sensor_origin.x() + ddxy; xx += m_res){
		for(double yy = sensor_origin.y() - ddxy; yy <= sensor_origin.y() + ddxy; yy += m_res){
			for(double zz = sensor_origin.z() - ddxy; zz <= sensor_origin.z() + ddxy; zz += m_res){
				octomap::point3d pp(xx, yy, zz);

				octomap::OcTreeNode* pp_node = octree->search(pp);
				if(pp_node != NULL){
					occ_p = octree->isNodeOccupied(pp_node);
				} else{
					occ_p = false;
				}

				octree->updateNode(pp, false);

				if(occ_p){
					octomap::OcTreeNode* pp_node_a = octree->search(pp);
					if(pp_node_a != NULL){
						occ_a = octree->isNodeOccupied(pp_node);
					} else{
						occ_a = false;
					}

					if(!occ_a){
						Eigen::Matrix<double, 3, 1> pa;
						pa(0) = pp.x();
						pa(1) = pp.y();
						pa(2) = pp.z();

						toBe_removed.push_back(pa);
						//checkToBeInserted(pa);
					}
				}
			}
		}
	}

	// Update Octomap
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = current_pc_full.begin(); iter != current_pc_full.end(); iter++){
		octomap::point3d pp(iter->x, iter->y, iter->z);

		octomap::KeyRay free_rays;
		octree->computeRayKeys(sensor_origin, pp, free_rays);

		for(octomap::KeyRay::iterator it = free_rays.begin(),
			end = free_rays.end()-1; it != end; it++){

			octomap::OcTreeNode* it_node = octree->search(*it);
			if(it_node != NULL){
				occ_p = octree->isNodeOccupied(it_node);
			} else{
				occ_p = false;
			}

			octree->updateNode(*it, false);

			if(occ_p){
				octomap::OcTreeNode* it_node_a = octree->search(*it);
				if(it_node_a != NULL){
				occ_a = octree->isNodeOccupied(it_node_a);
				} else{
				occ_a = false;
				}

				if(!occ_a){
					pp = octree->keyToCoord(*it);
					Eigen::Matrix<double, 3, 1> pa;
					pa(0) = pp.x();
					pa(1) = pp.y();
					pa(2) = pp.z();

					toBe_removed.push_back(pa);
					//checkToBeInserted(pa);
				}
			}
		}
	}

	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = insert_cloud.begin(); iter != insert_cloud.end(); iter++){
		octomap::point3d pp(iter->x, iter->y, iter->z);

		octomap::OcTreeNode* pp_node = octree->search(pp);
		if(pp_node != NULL){
			occ_p = octree->isNodeOccupied(pp_node);
		} else{
			occ_p = false;
		}

		octree->updateNode(pp, true);

		if(!occ_p){
			octomap::OcTreeNode* pp_node_a = octree->search(pp);
			if(pp_node_a != NULL){
				occ_a = octree->isNodeOccupied(pp_node_a);
			} else{
				occ_a = false;
			}

			if(occ_a){
				Eigen::Matrix<double, 3, 1> pa;
				pa(0) = pp.x();
				pa(1) = pp.y();
				pa(2) = pp.z();
				toBe_inserted.push_back(pa);
				checkToBeRemoved(pa);
			}
		}
	}

	// Publish the Update Octomap
	if(debug_flag){
		pcl::PointCloud<pcl::PointXYZRGB> octomap_pc;
		for(octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()),
			end = octree->end(); it != end; it++){
			
			if(it->getOccupancy() > occ_thr){
				double h = abs((it.getZ() - m_min_z)/(m_max_z - m_min_z))*color_factor;
				Eigen::Matrix<double, 4, 1> cc = heightColor(h);

				pcl::PointXYZRGB pp;
				pp.x = it.getX();
				pp.y = it.getY();
				pp.z = it.getZ();
				pp.r = cc(1);
				pp.g = cc(2);
				pp.b = cc(3);
				octomap_pc.push_back(pp);
			}
		}

		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg(octomap_pc, msg);
		msg.header.frame_id = "map";
		visualPublisher.publish(msg);
	}

	publishOctomap();
	
	octomap_updating = false;
}

bool Environment::saveMap(mav_planning::SaveMap::Request &req, mav_planning::SaveMap::Response &res){
	std::string ss(req.name);
	if(octree == NULL){
		ROS_WARN("[ENVIRONMENT]: Octree not Initialized");
		return false;
	}

	ROS_INFO("[ENVIRONMENT]: Saving Map, Size: %ld, Res: %f", octree->size(), octree->getResolution());
	std::string sx = ss.substr(ss.length()-3, 3);
	if(sx == ".bt"){
		if(!octree->writeBinary(ss))
			ROS_WARN("[ENVIRONMENT]: Error Writing Map");
	} else if(sx == ".ot"){
			if(!octree->write(ss))
				ROS_WARN("[ENVIRONMENT]: Error Writing Map");
	} else{
		ROS_WARN("[ENVIRONMENT]: Unknown Extension");
		return false;
	}

	ROS_INFO("[ENVIRONMENT]: Saving Map Done");
	return true;
}

void Environment::publishOctomap(){
	octomap_msgs::Octomap map;
	map.header.frame_id = "map";
	map.header.stamp = ros::Time::now();

	if(octomap_msgs::binaryMapToMsg(*octree, map))
		binaryPublisher.publish(map);
	else
		ROS_ERROR("[ENVIRONMENT]: Error Serializing OctoMap");

	if(octomap_msgs::fullMapToMsg(*octree, map))
		octreePublisher.publish(map);
	else
		ROS_ERROR("[ENVIRONMENT]: Error serializing OctoMap");
}

Eigen::Matrix<double, 4, 1> Environment::heightColor(double h){
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

void Environment::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud,
								const nav_msgs::Odometry::ConstPtr& odom){

	// Read & Save Current Transformation/Odometry
	#ifdef MAVROS
	try{
		tf2::fromMsg(_tfBuffer.lookupTransform("map", "odom", ros::Time(0)).transform, mapToOdom_transformation);
		transform_valid = true;
	} catch(tf2::TransformException &ex){
		pc_processing = false;
		transform_valid = false;
		return;
	}
	#endif
	#ifdef FLIGHTMARE
	tf2::Vector3 zero_translation(0.0, 0.0, 0.0);
	tf2::Quaternion zero_quaternion(0.0, 0.0, 0.0, 1.0);

	mapToOdom_transformation.setOrigin(zero_translation);
	mapToOdom_transformation.setRotation(zero_quaternion);
	transform_valid = true;
	#endif

	// Save Current Odometry
	tf2::Vector3 odomToBody_translation(odom->pose.pose.position.x,
										odom->pose.pose.position.y,
										odom->pose.pose.position.z);
										
	tf2::Quaternion odomToBody_quaternion(odom->pose.pose.orientation.x,
									      odom->pose.pose.orientation.y,
										  odom->pose.pose.orientation.z,
										  odom->pose.pose.orientation.w);

	odomToBody_transform.setRotation(odomToBody_quaternion);
	odomToBody_transform.setOrigin(odomToBody_translation);
	tf2::Transform maptoBody_transform = mapToOdom_transformation*odomToBody_transform;

	currentOdometry = *odom;
	odom_valid = true;

	// Start Callback Execution
	if(pc_processing){
		return;
	}

	pc_processing = true;

	Eigen::Matrix<double, 4, 4> pc_tranformation;
	tf2::Vector3 t = maptoBody_transform.getOrigin();
	tf2::Quaternion q = maptoBody_transform.getRotation();
	pc_tranformation << 1-(2*q.y()*q.y())-(2*q.z()*q.z()), 2*q.x()*q.y()-2*q.w()*q.z(), 2*q.x()*q.z()+2*q.w()*q.y(), t.x(),
						2*q.x()*q.y()+2*q.w()*q.z(), 1-(2*q.x()*q.x())-(2*q.z()*q.z()), 2*q.y()*q.z()-2*q.w()*q.x(), t.y(),
						2*q.x()*q.z()-2*q.w()*q.y(), 2*q.y()*q.z()+2*q.w()*q.x(), 1-(2*q.x()*q.x())-(2*q.y()*q.y()), t.z(),
						0.0, 0.0, 0.0, 1.0;

	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::fromROSMsg(*cloud, pc);

	if(pc.empty()){
		pc_processing = false;
		return;
	}

	#ifdef MAVROS
	pc_tranformation = pc_tranformation*sensor_toBody;
	#endif
	#ifdef FLIGHTMARE
	pc_tranformation = pc_tranformation*flightmare_rotation*sensor_toBody;
	#endif

	pcl::VoxelGrid<pcl::PointXYZ> ds;
	ds.setInputCloud(pc.makeShared());
	ds.setLeafSize(ds_r, ds_r, ds_r);
	ds.filter(pc);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(pc.makeShared());
	sor.setMeanK(mean_kkn);
	sor.setStddevMulThresh(std_dev);
	sor.filter(pc);

	// Clone PC for Deletion
	pcl::PassThrough<pcl::PointXYZ> pt_1;
	pt_1.setFilterFieldName("z");
	pt_1.setFilterLimits(min_r, maximum_dd);
	pt_1.setInputCloud(pc.makeShared());
	pt_1.filter(pc);

	if(pc.size() != 0){
		pcl::PointCloud<pcl::PointXYZ> pc_cc = pcl::PointCloud<pcl::PointXYZ>(pc);
		pcl::transformPointCloud(pc_cc, pc_cc, pc_tranformation);
		current_pc_full_ = pcl::PointCloud<pcl::PointXYZ>(pc_cc);
	} else{
		pc_processing = false;
		return;
	}

	// Cutoff at Sensor Max Distance
	pcl::PassThrough<pcl::PointXYZ> pt_2;
	pt_2.setFilterFieldName("z");
	pt_2.setFilterLimits(min_r, max_r);
	pt_2.setInputCloud(pc.makeShared());
	pt_2.filter(pc);

	pcl::transformPointCloud(pc, pc, pc_tranformation);

	pcl::PassThrough<pcl::PointXYZ> pt_x, pt_y, pt_z;
	pt_x.setFilterFieldName("x");
	pt_y.setFilterFieldName("y");
	pt_z.setFilterFieldName("z");
	pt_x.setFilterLimits(m_min_x, m_max_x);
	pt_y.setFilterLimits(m_min_y, m_max_y);
	pt_z.setFilterLimits(m_min_z, m_max_z);

	pt_x.setInputCloud(pc.makeShared());
	pt_x.filter(pc);

	pt_y.setInputCloud(pc.makeShared());
	pt_y.filter(pc);

	pt_z.setInputCloud(pc.makeShared());
	pt_z.filter(pc);

	// Save Current PointCloud
	if(pc.size() != 0){
		octomap::point3d origin(t.x(), t.y(), t.z());
		sensor_origin_ = origin;
		current_pc_ = pcl::PointCloud<pcl::PointXYZ>(pc);

		if(debug_flag){
			sensor_msgs::PointCloud2 msg_debug;
			pcl::toROSMsg(pc, msg_debug);
			msg_debug.header.frame_id = "map";
			debugPublisher.publish(msg_debug);
		}
	}

	pc_processing = false;
}

void Environment::checkToBeRemoved(Eigen::Matrix<double, 3, 1> pp){
	for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = toBe_removed.begin();
		iter != toBe_removed.end(); iter++){

		if((pp - (*iter)).norm() < 1e-3){
			toBe_removed.erase(iter);
			break;
		}
	}
}

void Environment::checkToBeInserted(Eigen::Matrix<double, 3, 1> pp){
	for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = toBe_inserted.begin();
		iter != toBe_inserted.end(); iter++){

		if((pp - (*iter)).norm() < 1e-3){
			toBe_inserted.erase(iter);
			break;
		}
	}
}

double Environment::evaluateSDF(double x, double y, double z){
	return sdf_map->getDistance(x,y,z);
}

double Environment::evaluateSDF_withGradient(double x, double y, double z, Eigen::Matrix<double, 3, 1>& grad){
	return sdf_map->getDistanceWithGradient(x,y,z,grad);
}

// SDF Functions /////////////////////////////////////////////////////////
SDFMap::SDFMap(ros::NodeHandle& nh):
	visPublisher(nh.advertise<visualization_msgs::Marker>("/vis_map", 1)){
	nh.param("map_name", map_name, std::string("/home/gent/catkin_ws/src/flightmare/flightrender/pointcloud_data/leonardo.ply"));
	nh.param("grid_name", grid_name, std::string("/home/gent/catkin_ws/src/mav_planning/data/leonardo.bin"));
	nh.param("map_resolution", m_res, 0.2);
	nh.param("max_x", m_max_x, 0.2);
	nh.param("max_y", m_max_y, 10.2);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, -0.2);
	nh.param("min_z", m_min_z, -0.2);
	nh.param("occ_thr", occ_thr, 0.5);
	nh.param("use_previous_map", use_map, true);
	nh.param("publish_debug_env", debug_flag, true);
	nh.param("max_nn_threads", max_nn_threads, 4);

	if(use_map){
		pcl::io::loadPLYFile(map_name, pc);
		kdtree.setInputCloud(pc.makeShared());
	}

	if(!loadGrid()){
		computeGrid();
		saveGrid();
	} else{
		// Wait a While for Map Publication
		ros::Duration(0.5).sleep();
	}

	if(debug_flag){
		publishMap();
	}
}

SDFMap::~SDFMap(){
	delete m_grid;
}

void SDFMap::update(std::vector<Eigen::Matrix<double, 3, 1>> insert, std::vector<Eigen::Matrix<double, 3, 1>> remove, octomap::OcTree* octree){
	if(insert.size() != 0){
		// Generate Insertion Point Cloud
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ> pi;
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = insert.begin();
			iter != insert.end(); iter++){
			
			pcl::PointXYZ pp;
			pp.x = (*iter)(0);
			pp.y = (*iter)(1);
			pp.z = (*iter)(2);

			pi.points.push_back(pp);
		}

		// Generate Insertion Kdtree
		ros::Time t2 = ros::Time::now();
		kdtree_u.setInputCloud(pi.makeShared());

		// Update Map -> MultiThread
		ros::Time t3 = ros::Time::now();
		std::vector<std::thread> threads;
		int dii = m_gridSize_z/max_nn_threads;
		int res = m_gridSize_z%max_nn_threads;

		for(int ii = 0; ii < max_nn_threads; ii++){
			if(ii == max_nn_threads-1){
				threads.emplace_back(&SDFMap::updateWithInsertion, this, ii*dii, (ii+1)*dii + res, kdtree_u);
			} else{
				threads.emplace_back(&SDFMap::updateWithInsertion, this, ii*dii, (ii+1)*dii, kdtree_u);
			}
		}

		for(int ii = 0; ii < max_nn_threads; ii++){
			threads[ii].join();
		}

		ros::Time t4 = ros::Time::now();
		ROS_INFO_COND(debug_flag, "[ENVIRONMENT]: Update Times PC-GEN %f, KD-GEN %f, MAP-UPDATE %f", (t2-t1).toSec(), (t3-t2).toSec(), (t4-t3).toSec());
	}

	if(remove.size() != 0){
		// Generate Octomap PointCloud
		pcl::PointCloud<pcl::PointXYZ> octomap_pc;
		for(octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()),
			end = octree->end(); it != end; it++){
			
			if(it->getOccupancy() > occ_thr){
				pcl::PointXYZ pp;
				pp.x = it.getX();
				pp.y = it.getY();
				pp.z = it.getZ();

				octomap_pc.push_back(pp);
			}
		}

		// Generate Removing PointCloud
		pcl::PointCloud<pcl::PointXYZ> pr;
		for(std::vector<Eigen::Matrix<double, 3, 1>>::iterator iter = remove.begin();
			iter != remove.end(); iter++){
			
			pcl::PointXYZ pp;
			pp.x = (*iter)(0);
			pp.y = (*iter)(1);
			pp.z = (*iter)(2);

			pr.points.push_back(pp);
		}

		// Generate Kdtrees
		kdtree_u.setInputCloud(pr.makeShared());
		kdtree_m.setInputCloud(octomap_pc.makeShared());

		// Update Map -> MultiThread
		std::vector<std::thread> threads;
		int dii = m_gridSize_z/max_nn_threads;
		int res = m_gridSize_z%max_nn_threads;

		for(int ii = 0; ii < max_nn_threads; ii++){
			if(ii == max_nn_threads-1){
				threads.emplace_back(&SDFMap::updateWithInsertion, this, ii*dii, (ii+1)*dii + res, kdtree_u);
			} else{
				threads.emplace_back(&SDFMap::updateWithInsertion, this, ii*dii, (ii+1)*dii, kdtree_u);
			}
		}

		for(int ii = 0; ii < max_nn_threads; ii++){
			threads[ii].join();
		}
	}
}

void SDFMap::updateWithInsertion(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt){
	for(int iz = init_z; iz < end_z; iz++){
		for(int iy = 0; iy < m_gridSize_y; iy++){
			for(int ix = 0; ix < m_gridSize_x; ix++){
				pcl::PointXYZ searchPoint;
				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);

				searchPoint.x = ix*m_res + m_min_x;
				searchPoint.y = iy*m_res + m_min_y;
				searchPoint.z = iz*m_res + m_min_z;
				int index = ix + iy*m_gridStep_y + iz*m_gridStep_z;

				if(index >= m_gridSize){
					continue;
				}

				uint16_t dd = 0;
				if(kt.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
					dd = (uint16_t)(sqrt(pointNKNSquaredDistance[0])*100.0);
					m_grid[index] = std::min(m_grid[index], dd);
				}
			}
		}
	}
}

void SDFMap::updateWithDeletion(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt){
	for(int iz = init_z; iz < end_z; iz++){
		for(int iy = 0; iy < m_gridSize_y; iy++){
			for(int ix = 0; ix < m_gridSize_x; ix++){
				pcl::PointXYZ searchPoint;
				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);

				searchPoint.x = ix*m_res + m_min_x;
				searchPoint.y = iy*m_res + m_min_y;
				searchPoint.z = iz*m_res + m_min_z;
				int index = ix + iy*m_gridStep_y + iz*m_gridStep_z;

				if(index >= m_gridSize){
					continue;
				}

				uint16_t dd = 0;
				uint16_t da = 0;
				if(kt.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
					dd = (uint16_t)(sqrt(pointNKNSquaredDistance[0])*100.0);
					da = m_grid[index];

					if(dd == da){
						m_grid[index] = (uint16_t)(getDistanceFromMap(searchPoint)*100.0);
					}
				}
			}
		}
	}
}

double SDFMap::getDistanceFromMap(pcl::PointXYZ pp){
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	if(use_map){
		kdtree.nearestKSearch(pp, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		double d1 = sqrt(pointNKNSquaredDistance[0]);

		kdtree_m.nearestKSearch(pp, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		double d2 = sqrt(pointNKNSquaredDistance[0]);

		return std::min(d1, d2);

	} else{
		kdtree_m.nearestKSearch(pp, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		return sqrt(pointNKNSquaredDistance[0]);
	}
}

double SDFMap::getDistance(double px, double py, double pz){
	if(isIntoMap(px, py, pz)){
		uint64_t ii = point2grid(px, py, pz); 
		return (double)m_grid[ii]*0.01;
	}

	return 0.0;
}

double SDFMap::getDistanceWithGradient(double px, double py, double pz, Eigen::Matrix<double, 3, 1>& grad){
	TrilinearParameters tp = getTrilinearParameters(px, py, pz);
	double dd = tp.p1 + tp.p2*px + tp.p3*py + tp.p4*pz + tp.p5*px*py + tp.p6*px*pz + tp.p7*py*pz + tp.p8*px*py*pz;

	grad(0) = (tp.p2 + tp.p6*pz + tp.p5*py + tp.p8*pz*py);
	grad(1) = (tp.p3 + tp.p7*pz + tp.p5*px + tp.p8*pz*px);
	grad(2) = (tp.p4 + tp.p6*px + tp.p7*py + tp.p8*px*py);

	return dd;
}

TrilinearParameters SDFMap::getTrilinearParameters(double px, double py, double pz){
	TrilinearParameters tp;
	if(isIntoMap(px, py, pz)){
		uint64_t ii = point2grid(px, py, pz); 

		double c000 = (double)m_grid[ii]*0.01; 
		double c001 = (double)m_grid[ii + m_gridStep_z]*0.01; 
		double c010 = (double)m_grid[ii + m_gridStep_y]*0.01; 
		double c011 = (double)m_grid[ii + m_gridStep_y + m_gridStep_z]*0.01; 
		double c100 = (double)m_grid[ii + 1]*0.01; 
		double c101 = (double)m_grid[ii + 1 + m_gridStep_z]*0.01; 
		double c110 = (double)m_grid[ii + 1 + m_gridStep_y]*0.01; 
		double c111 = (double)m_grid[ii + 1 + m_gridStep_y + m_gridStep_z]*0.01; 

		double div = -1.0/(m_res*m_res*m_res);
		double x0 = ((int)(px/m_res))*m_res;
		double x1 = x0 + m_res;
		double y0 = ((int)(py/m_res))*m_res;
		double y1 = y0 + m_res;
		double z0 = ((int)(pz/m_res))*m_res;
		double z1 = z0 + m_res;

		tp.p1 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 + c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
		tp.p2 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0 - c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
		tp.p3 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 - c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
		tp.p4 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 - c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
		tp.p5 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 - c101*z0 - c110*z1 + c111*z0)*div;
		tp.p6 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 - c101*y1 - c110*y0 + c111*y0)*div;
		tp.p7 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 - c101*x0 - c110*x0 + c111*x0)*div;
		tp.p8 = (c000 - c001 - c010 + c011 - c100 + c101 + c110 - c111)*div;
	}

	return tp;
}

void SDFMap::saveGrid(){
	FILE* ff;
		
	// Open File
	ff = fopen(grid_name.c_str(), "wb");
	if(ff == NULL){
		ROS_ERROR("[GRID MAP]: Error opening file");
		return;
	}
	
	fwrite(&m_gridSize, sizeof(int), 1, ff);
	fwrite(&m_gridSize_x, sizeof(int), 1, ff);
	fwrite(&m_gridSize_y, sizeof(int), 1, ff);
	fwrite(&m_gridSize_z, sizeof(int), 1, ff);
	
	// Write Grid Cells
	fwrite(m_grid, sizeof(uint16_t), m_gridSize, ff);
	
	// Close File
	fclose(ff);

	ROS_INFO("[ENVIRONMENT]: Grid Saved!");
}

bool SDFMap::loadGrid(){
	FILE* ff;
	
	// Open File
	ff = fopen(grid_name.c_str(), "rb");
	if(ff == NULL){
		ROS_WARN("[ENVIRONMENT]: Grid not Found - Start Generation");
		return false;
	}
		
	fread(&m_gridSize, sizeof(int), 1, ff);
	fread(&m_gridSize_x, sizeof(int), 1, ff);
	fread(&m_gridSize_y, sizeof(int), 1, ff);
	fread(&m_gridSize_z, sizeof(int), 1, ff);

	m_gridStep_y = m_gridSize_x;
	m_gridStep_z = m_gridSize_x*m_gridSize_y;
	m_grid = new uint16_t[m_gridSize];
	fread(m_grid, sizeof(uint16_t), m_gridSize, ff);		
	fclose(ff);
	
	ROS_INFO("[ENVIRONMENT]: Grid Loaded From File");
	return true;
}

void SDFMap::computeGrid(){
	// Alloc the 3D Grid
	m_gridSize_x = (int)((m_max_x - m_min_x)/m_res);
	m_gridSize_y = (int)((m_max_y - m_min_y)/m_res); 
	m_gridSize_z = (int)((m_max_z - m_min_z)/m_res);
	m_gridSize   = m_gridSize_x*m_gridSize_y*m_gridSize_z;
	m_gridStep_y = m_gridSize_x;
	m_gridStep_z = m_gridSize_x*m_gridSize_y;
	m_grid = new uint16_t[m_gridSize];

	// Compute the Distance to the Closest Point of the Grid
	std::vector<std::thread> threads;
	int dii = m_gridSize_z/max_nn_threads;
	int res = m_gridSize_z%max_nn_threads;

	for(int ii = 0; ii < max_nn_threads; ii++){
		if(ii == max_nn_threads-1){
			threads.emplace_back(&SDFMap::computePartialGrid, this, ii*dii, (ii+1)*dii + res, kdtree);
		} else{
			threads.emplace_back(&SDFMap::computePartialGrid, this, ii*dii, (ii+1)*dii, kdtree);
		}
	}

	for(int ii = 0; ii < max_nn_threads; ii++){
		threads[ii].join();
	}
}

void SDFMap::computePartialGrid(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt){
	// Compute the Distance to the Closest Point of the Grid
	for(int iz = init_z; iz < end_z; iz++){
		pcl::PointXYZ searchPoint;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		ROS_INFO("[ENVIRONMENT]: Processing z = %d out of %d", iz, end_z);
		for(int iy = 0; iy < m_gridSize_y; iy++){
			for(int ix = 0; ix < m_gridSize_x; ix++){
				searchPoint.x = ix*m_res + m_min_x;
				searchPoint.y = iy*m_res + m_min_y;
				searchPoint.z = iz*m_res + m_min_z;
				int index = ix + iy*m_gridStep_y + iz*m_gridStep_z;

				if(use_map){
					if(kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
						m_grid[index] = (uint16_t)(sqrt(pointNKNSquaredDistance[0])*100.0);
					} else{
						m_grid[index] = 0;
					}
				} else{
					m_grid[index] = 1000;
				}
			}
		}
	}
}

inline int SDFMap::point2grid(const float &x, const float &y, const float &z){
	return (int)((x - m_min_x)/m_res) + (int)((y - m_min_y)/m_res)*m_gridStep_y + (int)((z - m_min_z)/m_res)*m_gridStep_z;
}

inline bool SDFMap::isIntoMap(double x, double y, double z){
	return (x >= m_min_x && y >= m_min_y && z >= m_min_z && x < m_max_x && y < m_max_y && z < m_max_z);
}

void SDFMap::publishMap(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Map";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.id += 1;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.scale.x = 0.03;
	marker.scale.y = 0.03;
	marker.scale.z = 0.03;
	marker.color.b = 1.0;
	marker.color.r = 0.0;
	marker.color.a = 1.0;

	geometry_msgs::Point p;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = pc.begin();
		iter != pc.end(); iter++){

		p.x = iter->x;
		p.y = iter->y;
		p.z = iter->z;

		marker.points.push_back(p);
	}

	visPublisher.publish(marker);
}