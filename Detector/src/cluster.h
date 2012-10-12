#if !defined(_CLUSTER_H_)
#define _CLUSTER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include "Detector/Detection.h"
#include "OnlineBoosting/Regions.h"

using namespace std;
using namespace Detector;

extern double HOG_THRESHOLD;
namespace Detector{
class Cluster {
	typedef pcl::PointXYZ PointType;
	float *height;
	float *distance;
	double confidence;
  float *max_y;
	std_msgs::Header *header;
	Eigen::Vector4f *centroid;
	pcl::PointCloud<PointType>::Ptr cloud_cluster;
  cv::Mat *blob;
	cv_bridge::CvImagePtr cv_ptr;
	Rect blobROI;
  visualization_msgs::Marker *boundingBox;
	Detector::Detection *detection;

  public:
    Cluster (float, pcl::PointCloud<PointType>::Ptr&, float, float, const std_msgs::Header&);
    ~Cluster (); // Destructor

		void removeZeroPoints (void);
		void CalculateConfidence (void);
    void CalculateOclusion(void);
		void ShowBlobInWindow(std::string window_name);
    void produceRGBBlob(const sensor_msgs::CameraInfoConstPtr& camInfo, const sensor_msgs::ImageConstPtr& image);
		visualization_msgs::Marker Mark_cluster(void);
	  void CalculateCentroid(void);
		Rect getTrackingROI(float searchFactor, Rect validROI);
	
		vector<float> getThisPeopleDetector2(void);
		Rect getROIofBlob(void){
			return blobROI;
		};
		double getConfidence(void){
			return confidence;
		};

		cv_bridge::CvImagePtr getCamImg(void){
			return cv_ptr;
		};
		Detector::Detection* Create_msg(void);
  	pcl::PointCloud<PointType>::Ptr ReturnCluster(void){
			return cloud_cluster;
    };


    float ReturnHeight(void){
			return *height;
		};

		Eigen::Vector4f ReturnCentroid(void){
			return *centroid;
		}
	  
};

/*Contructor*/
Cluster::Cluster(float input_height, pcl::PointCloud<PointType>::Ptr& input_cluster, float input_distance, float input_max_y, const std_msgs::Header &headerIn)
  {
  height = new float;
  distance = new float;
  max_y = new float;
	header = new std_msgs::Header(headerIn);
  boundingBox = new visualization_msgs::Marker();
  detection = new Detector::Detection();
  centroid = new Eigen::Vector4f();
  confidence = -100.0;
  *max_y = input_max_y;
	*height = input_height;
	cloud_cluster = input_cluster;
	*distance = input_distance;
  
}

/*Destructor*/
Cluster::~Cluster () {
  delete height;
	delete distance;
  delete max_y;
	delete centroid;
	if(blob)
  	delete blob;
	delete header;
  delete detection;
  delete boundingBox;
}
}
#endif
