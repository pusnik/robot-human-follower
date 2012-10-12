#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/console/parse.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cv_bridge/cv_bridge.h>

#include <Detector/Detection.h>
#include <Detector/DetectionList.h>
#include <Detector/prediction.h>
#include <time.h>

#include <boost/thread.hpp>

#include "cluster.cpp"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "OnlineBoosting/ImageRepresentation.h"
#include "OnlineBoosting/Patches.h"
#include "OnlineBoosting/StrongClassifier.h"
#include "OnlineBoosting/StrongClassifierDirectSelection.h"
#include "OnlineBoosting/BoostingDetector.h"

namespace Detector
{
class Detection_Tracking : public nodelet::Nodelet{
	private:
		StrongClassifier* classifier;
		Detector::Rect trackedPatch;
		Detector::Size img_size;
		bool initialized_boosting;
		BoostingDetector* detector;
		

  public:
		visualization_msgs::Marker *track_mark;
		list<Cluster>* clusters;
		ImageRepresentation* wholeImage;
		ros::Publisher publish_clusters;
		ros::Publisher detections;
    ros::Publisher published_marker;
		ros::Publisher publish_blob;
		int prediction_counter;
		int png_name_counter;

		volatile bool running;
		bool found;
		ros::NodeHandlePtr nh;
	
		//ADABOOST var
		int numWeakClassifier;
		int numBaseClassifier;
		bool useFeatureExchange;
	  int iterationInit;
	
		boost::mutex mtx_;
		typedef pcl::PointXYZ PointType;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
		message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud;
		message_filters::Subscriber<sensor_msgs::Image> camera_rgb;
		message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info;
		typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;
		boost::shared_ptr<Synchronizer> sync;

		double area_min_x;
		double area_max_x;
		double area_min_y;
		double area_max_y;
		double area_min_z;
		double area_max_z;

		Eigen::Vector4f min;
		Eigen::Vector4f max;

		bool PUB_CLUSTERS;
		bool PUB_BLOBS;
    double DOWNSAMPLING_LEAF_SIZE;
		int RANSAC_MAX_ITERATIONS;
		double RANSAC_DISTANCE_THRESHOLD;
		double CLUSTER_TOLERANCE;
		int CLUSTER_MIN_SIZE;
		int CLUSTER_MAX_SIZE;
		double MAX_ALLOWED_CLUSTER_DEPTH;
		double MIN_ALLOWED_CLUSTER_DEPTH;
		double MAX_ALLOWED_CLUSTER_HEIGHT;
		double MIN_ALLOWED_CLUSTER_HEIGHT;
		double MAX_ALLOWED_CLUSTER_WIDTH;
		double MIN_ALLOWED_CLUSTER_WIDTH;
		double ADABOOST_THRESHOLD;
		double ADABOOST_SEARCH_AREA;
		double ADABOOST_INIT_STEPS;
		double ADABOOST_NUM_BASE_CLASS;
		double AREA_X;
		double AREA_Y;
		double AREA_Z;
		int MAX_FRAMES_WITHOUT_HOG_CONFIRMATION;
		int MAX_PREDICTIONS;
		int frame_counter;
    
		virtual void onInit();
		~Detection_Tracking();
    void cloud_preprocessing (const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
    void publish_blobs(list<Cluster>* clusters, const sensor_msgs::ImageConstPtr& image, const std_msgs::Header header);
    void publish_pc_clusters(list<Cluster>* clusters, std::string frame_id, ros::Time stamp, bool show_box);
    void publish_height (float x, float y, float z, std::string msg, std::string frame_id);
    /*Calculate height of a cluster*/
		float calculate_height(float a, float b, float c, float d, float x, float y, float z){
			return std::fabs((a * x) + (b * y) + (c * z) + d) / sqrt((a * a) + (b * b) + (c * c));	
		};
    void publish_rgb_blobs(list<Cluster>* clusters, const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const std_msgs::Header header);
		void Mark_path(double x, double y, double z, int r, int g, int b);
		void make_command(geometry_msgs::PointStampedPtr& worldPoint);
		void calculate_search_window(float x, float y, float z);
		geometry_msgs::PointStamped getWorldPoint(geometry_msgs::PointStampedPtr& p, const char *frame);
		Detector::prediction::Response getKalmanPrediction();
		void Mark_search_box(void);
};
}

