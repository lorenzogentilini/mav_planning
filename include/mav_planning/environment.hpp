#pragma once
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <thread>
#include <chrono>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <mav_planning/SaveMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define MAVROS
//#define FLIGHTMARE

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

class TrilinearParameters{
	public:
	TrilinearParameters(){
		p1 = p2 = p3 = p4 = p5 = p6 = p7 = p8 = 0.0;
	};

	inline double interpolate(double x, double y, double z){
		return p1 + p2*x + p3*y + p4*z + p5*x*y + p6*x*z + p7*y*z + p8*x*y*z;
	};

	double p1, p2, p3, p4, p5, p6, p7, p8;
};

class SDFMap{
	public:
	// Class Constructor
	SDFMap(ros::NodeHandle& nh);

	// Class Destructor
	~SDFMap();

	double getDistance(double px, double py, double pz);
	double getDistanceWithGradient(double px, double py, double pz, Eigen::Matrix<double, 3, 1>& grad);
	void update(std::vector<Eigen::Matrix<double, 3, 1>> insert, std::vector<Eigen::Matrix<double, 3, 1>> remove, octomap::OcTree* octree);
	TrilinearParameters getTrilinearParameters(double px, double py, double pz);

	private:
	std::string map_name, grid_name;
	bool use_map, debug_flag;

	int m_gridSize_x, m_gridSize_y, m_gridSize_z;
	int m_gridSize, m_gridStep_y, m_gridStep_z;
	double m_max_x, m_max_y, m_max_z, m_min_x, m_min_y, m_min_z;
	double occ_thr, m_res;
	uint16_t *m_grid;

	int max_nn_threads;
	const uint16_t maxValue = 1000;

	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_u;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_m;
	ros::Publisher visPublisher;

	void publishMap();
	bool loadGrid();
  	void saveGrid();
	void computeGrid();
  	void computePartialGrid(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt);
	double getDistanceFromMap(pcl::PointXYZ pp);
	void updateWithInsertion(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt);
	void updateWithDeletion(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt);
	inline int point2grid(const float &x, const float &y, const float &z);
	inline bool isIntoMap(double x, double y, double z);
};

class Environment{
	private:
	double m_res, p_hitting, p_missing, min_thr, max_thr, occ_thr;
	double ps_x, ps_y, ps_z, qs_x, qs_y, qs_z, qs_w;
	double m_max_x, m_max_y, m_max_z, m_min_x, m_min_y, m_min_z;
	double min_r, max_r, insertion_margin, ds_r, std_dev, color_factor;
	double maximum_dd;
	
	int mean_kkn;
	bool use_map, debug_flag;

	bool sdf_updating = false;
	bool octomap_updating = false;
	bool octomap_stopper = false;
	bool pc_processing = false;

	bool odom_valid = false;
	bool transform_valid = false;

	octomap::OcTree* octree;
	octomap::point3d sensor_origin_;

	pcl::PointCloud<pcl::PointXYZ> current_pc_;
	pcl::PointCloud<pcl::PointXYZ> current_pc_full_;
	pcl::PointCloud<pcl::PointXYZ> pc_map;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	Eigen::Matrix<double, 4, 4> sensor_toBody;
	Eigen::Matrix<double, 4, 4> flightmare_rotation;
	std::string mapName;

	std::vector<Eigen::Matrix<double, 3, 1>> toBe_inserted;
	std::vector<Eigen::Matrix<double, 3, 1>> toBe_removed;

	SDFMap* sdf_map;

	tf2_ros::Buffer _tfBuffer;
	tf2_ros::TransformListener _tfListener;
	tf2::Transform mapToOdom_transformation;
	tf2::Transform odomToBody_transform;
	nav_msgs::Odometry currentOdometry;

	ros::Publisher octreePublisher;
	ros::Publisher binaryPublisher;
	ros::Publisher visualPublisher;
	ros::Publisher debugPublisher;
	ros::Timer updateOctomap_timer;
	ros::Timer updateSdf_timer;
	ros::ServiceServer saveServer;

	message_filters::Subscriber<sensor_msgs::PointCloud2> depthSubscriber;
	message_filters::Subscriber<nav_msgs::Odometry> odomSubscriber;
	message_filters::Synchronizer<syncPolicy> sync;

	// Functions
	Eigen::Matrix<double, 4, 1> heightColor(double h);
	void checkToBeInserted(Eigen::Matrix<double, 3, 1> pp);
	void checkToBeRemoved(Eigen::Matrix<double, 3, 1> pp);

	bool saveMap(mav_planning::SaveMap::Request &req, mav_planning::SaveMap::Response &res);
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom);
	void updateOctomap(const ros::TimerEvent& e);
	void updateSDF(const ros::TimerEvent& e);

	void publishOctomap();

	public:
	// Class Constructor
	Environment(ros::NodeHandle& nh);

	// Class Destructor
	~Environment();

	double evaluateSDF(double x, double y, double z);
	double evaluateSDF_withGradient(double x, double y, double z, Eigen::Matrix<double, 3, 1>& grad);
	pcl::PointCloud<pcl::PointXYZ> getLastCloud(){ return current_pc_; };
	
	bool getLastOdometry(nav_msgs::Odometry& odom){
		if(!odom_valid){
			return false;
		}

		odom = currentOdometry;
		return true;
	};

	bool getLastTransform(tf2::Transform& tf){
		if(!transform_valid){
			return false;
		}
		
		tf = mapToOdom_transformation;
		return true;
	};


	bool getLastOdomTransform(tf2::Transform& tf){
		if(!odom_valid){
			return false;
		}
		
		tf = odomToBody_transform;
		return true;
	};
};