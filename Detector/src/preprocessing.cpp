#include "preprocessing.h"
#include <pluginlib/class_list_macros.h>

using namespace std;

///robot_pose_ekf/odom
double HOG_THRESHOLD = 0.0;
bool have_human = false;
double human_min_x;
double human_max_x;
double human_min_y;
double human_max_y;
double human_min_z;
double human_max_z;

double patch_left;
double patch_upper;
double patch_width;
double patch_height;
bool have_new_patch = false;
bool have_search_area = false;


Eigen::Vector4f min_kinect_bound;
Eigen::Vector4f max_kinect_bound;

PLUGINLIB_DECLARE_CLASS(Detector, Detection_Tracking, Detector::Detection_Tracking, nodelet::Nodelet); 

namespace Detector
{
/*Constructor*/
void Detection_Tracking::onInit(){  
		nh.reset(new ros::NodeHandle(getNodeHandle()));
		frame_counter = 0;	
		prediction_counter = 0;
		found = false;
		initialized_boosting = false;
		png_name_counter = 0;
		//Setup parameters from launch file
  	//std::cout << "Setting parameters from launch file:" << std::endl;

		nh->param("Detection_Tracking/downsample_leaf_size", DOWNSAMPLING_LEAF_SIZE, 0.06);
		nh->param("Detection_Tracking/ransac_distance_threshold", RANSAC_DISTANCE_THRESHOLD, 0.06);
		nh->param("Detection_Tracking/ransac_max_iterations", RANSAC_MAX_ITERATIONS, 250);
		nh->param("Detection_Tracking/cluster_tolerance", CLUSTER_TOLERANCE, 0.12);
		nh->param("Detection_Tracking/max_allowed_cluster_depth", MAX_ALLOWED_CLUSTER_DEPTH, 6.5);
		nh->param("Detection_Tracking/min_allowed_cluster_depth", MIN_ALLOWED_CLUSTER_DEPTH, 1.3);
		nh->param("Detection_Tracking/max_allowed_cluster_height", MAX_ALLOWED_CLUSTER_HEIGHT, 2.2);
		nh->param("Detection_Tracking/min_allowed_cluster_height", MIN_ALLOWED_CLUSTER_HEIGHT, 1.4);
		nh->param("Detection_Tracking/max_allowed_cluster_width", MAX_ALLOWED_CLUSTER_WIDTH, 0.9);
		nh->param("Detection_Tracking/min_allowed_cluster_width", MIN_ALLOWED_CLUSTER_WIDTH, 0.5);

		nh->param("Detection_Tracking/cluster_min_size", CLUSTER_MIN_SIZE, 200);
		nh->param("Detection_Tracking/cluster_max_size", CLUSTER_MAX_SIZE, 700);
		nh->param("Detection_Tracking/publish_blobs", PUB_BLOBS, true);
		nh->param("Detection_Tracking/publish_clusters", PUB_CLUSTERS, false);
		nh->param("Detection_Tracking/hog_threshold", HOG_THRESHOLD, -0.0);
		nh->param("Detection_Tracking/adaboost_threshold", ADABOOST_THRESHOLD, 0.0);
		nh->param("Detection_Tracking/adaboost_search_area", ADABOOST_SEARCH_AREA, 2.0);
		nh->param("Detection_Tracking/adaboost_init_steps", ADABOOST_INIT_STEPS, 50.0);
		nh->param("Detection_Tracking/adaboost_num_base_class", ADABOOST_NUM_BASE_CLASS, 100.0);

		nh->param("Detection_Tracking/area_x", AREA_X, 0.50);
		nh->param("Detection_Tracking/area_y", AREA_Y, 0.50);
		nh->param("Detection_Tracking/area_z", AREA_Z, 0.50);

		nh->param("Detection_Tracking/max_predictions", MAX_PREDICTIONS, 30);
		nh->param("Detection_Tracking/max_frames_withouth_hog_confirmation", MAX_FRAMES_WITHOUT_HOG_CONFIRMATION, 10);
		
		std::cout << "	DOWNSAMPLING LEAF SIZE: " << DOWNSAMPLING_LEAF_SIZE <<std::endl;
		std::cout << "	RANSAC DISTANCE THRESHOLD: " << RANSAC_DISTANCE_THRESHOLD <<std::endl;
		std::cout << "	MAX RANSAC ITERATIONS: " << RANSAC_MAX_ITERATIONS <<std::endl;
		std::cout << "	CLUSTER TOLERANCE: " << CLUSTER_TOLERANCE <<std::endl;
		std::cout << "	MAX ALLOWED CLUSTER DEPTH: " << MAX_ALLOWED_CLUSTER_DEPTH <<std::endl;
		std::cout << "	MIN ALLOWED CLUSTER DEPTH: " << MIN_ALLOWED_CLUSTER_DEPTH <<std::endl;
		std::cout << "	MAX ALLOWED CLUSTER HEIGHT: " << MAX_ALLOWED_CLUSTER_HEIGHT <<std::endl;
		std::cout << "	MIN ALLOWED CLUSTER HEIGHT: " << MIN_ALLOWED_CLUSTER_HEIGHT <<std::endl;
		std::cout << "	MAX ALLOWED CLUSTER WIDTH: " << MAX_ALLOWED_CLUSTER_WIDTH <<std::endl;
		std::cout << "	MIN ALLOWED CLUSTER WIDTH: " << MIN_ALLOWED_CLUSTER_WIDTH <<std::endl;
		std::cout << "	MAX CLUSTER SIZE: " << CLUSTER_MAX_SIZE <<std::endl;
		std::cout << "	MIN CLUSTER SIZE: " << CLUSTER_MIN_SIZE <<std::endl;
		std::cout << "	HOG THRESHOLD: " << HOG_THRESHOLD <<std::endl;
		std::cout << "	ADABOOST THRESHOLD: " << ADABOOST_THRESHOLD <<std::endl;
		std::cout << "	ADABOOST SEARCH AREA MULTIPLIER: " << ADABOOST_SEARCH_AREA <<std::endl;
		std::cout << "	ADABOOST INIT STEPS: " << ADABOOST_INIT_STEPS <<std::endl;
		std::cout << "	ADABOOST NUMBER OF BASE CLASSIFIERS: " << ADABOOST_NUM_BASE_CLASS <<std::endl;
		std::cout << "	PUBLISH BLOBS: " << PUB_BLOBS <<std::endl;
		std::cout << "	PUBLISH CLUSTERS: " << PUB_CLUSTERS <<std::endl;
		std::cout << "	SEARCH AREA X: " << AREA_X <<std::endl;
		std::cout << "	SEARCH AREA Y: " << AREA_Y <<std::endl;
		std::cout << "	SEARCh AREA Z: " << AREA_Z <<std::endl;
		std::cout << "	MAX FRAMES WITHOUT HOG CONFIRMATION: " << MAX_FRAMES_WITHOUT_HOG_CONFIRMATION <<std::endl;
		std::cout << "	MAX PREDICTIONS: " << MAX_PREDICTIONS <<std::endl;
		std::cout << "----------------------------------" << std::endl;
		std::cout << "Subscribing to topics" << std::endl;

		//ADABOOST var init
		numBaseClassifier = ADABOOST_NUM_BASE_CLASS;
		numWeakClassifier = numBaseClassifier*10;		
		useFeatureExchange = true;
	  iterationInit = ADABOOST_INIT_STEPS;
		img_size.width = 640;
		img_size.height = 480;
		
		
		//pointcloud.subscribe(*nh, "camera/depth/points", 1);
		pointcloud.subscribe(*nh, "clusters", 1);
		camera_rgb.subscribe(*nh, "camera/rgb/image_color", 1);
		camera_info.subscribe(*nh, "camera/rgb/camera_info", 1);
		sync.reset(new Synchronizer(MySyncPolicy(8), pointcloud, camera_rgb, camera_info));
		sync->registerCallback(boost::bind(&Detection_Tracking::cloud_preprocessing, this, _1, _2, _3));

		publish_clusters = nh->advertise<sensor_msgs::PointCloud2> ("debug",1);
		published_marker = nh->advertise<visualization_msgs::Marker> ("markers",0);
		publish_blob = nh->advertise<sensor_msgs::Image> ("blobs",1);
		detections = nh->advertise<Detector::DetectionList> ("detections",1);

		//std::cout << "Publishing topics" << std::endl;
		//std::cout << "Detection ready to start." << std::endl;
		//std::cout << "----------------------------------" << std::endl;	
		}

Detector::prediction::Response Detection_Tracking::getKalmanPrediction(){
	Detector::prediction::Request req;	
  Detector::prediction::Response res;	
	req.str = std::string("Daj predikcijo!");

	ros::service::waitForService("prediction");
	while(true){
		if (ros::service::call("prediction", req, res))
		{
			return res;
		}
	}
}


/* Check found clusters if any corresponds to human. For first human detection HOG from OpenCV is used,
 * and for all later classifications we use more robust Online Adaboost
 */
void Detection_Tracking::publish_rgb_blobs(list<Cluster>* clusters, const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const std_msgs::Header header){
	//std::cout << "Zacetek publish_rgb_blobs" << std::endl;
	list<Cluster>::iterator it;
	Rect validROI(0, 0, 480,640);
  Detector::DetectionList detectionMsg;

  /* For the time when we don't have human we evaluate each cluster with HOG
   * If we pass the threshold we take the most probable cluster as human and
   * on it initialize Online Adaboost classifier.
   */
	if(!have_human){
		if(!(clusters->empty()))
		{
		  double best_conf = -200;
		  list<Cluster>::iterator best;
			for(it = clusters->begin(); it != clusters->end(); it++) {	
				////std::cout << "Pred blob "<< std::endl;
				it->produceRGBBlob(cam_info, image);	
				////std::cout << "Po blob "<< std::endl;
				it->CalculateConfidence();
				////std::cout << "zdracunal Conf "<< std::endl;
				if(it->getConfidence() > best_conf){
						best_conf = it->getConfidence();
						best = it;
				}
			}
			//after checking all clusters, check if most confident of them is human
			if (best_conf >= HOG_THRESHOLD){
				//std::cout << "Imamo cloveka. "<< best_conf << std::endl;
				//std::cout <<  (best->ReturnCluster())->points.size()<< std::endl;
				//INIT ADABOOST
				
				have_human = true;
				best->CalculateCentroid();
				best->ShowBlobInWindow("1");
			
        //Calculate bouding box of found human
				pcl::getMinMax3D (*(best->ReturnCluster()), min, max);
				human_min_x = min[0];
				human_max_x = max[0];
				human_min_y = min[1];
				human_max_y = max[1];
				human_min_z = min[2];
				human_max_z = max[2];
        
				
				if(initialized_boosting == false){
					//Init Adaboost strong classifier
					std::cout << "Init adaboost "<< std::endl;
					Detector::Size patchSize;
					patchSize = best->getROIofBlob();
					std::cout << "Dobimo ROI detekcije"<< std::endl;
					classifier = new StrongClassifierDirectSelection(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);
				
					detector = new BoostingDetector (classifier);
					trackedPatch = best->getROIofBlob();
					Detector::Rect trackingROI = best->getTrackingROI(ADABOOST_SEARCH_AREA, validROI);			
					Detector::Size trackedPatchSize;
					trackedPatchSize = trackedPatch;		
					std::cout << "1 " << std::endl;		
					Patches *trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);
					//to GrayScale
					std::cout << "2" << std::endl;	
					cv::Mat im_rgb = (best->getCamImg())->image;
					cv::Mat im_gray;
					cv::cvtColor(im_rgb,im_gray,CV_RGB2GRAY);
					unsigned char *pixels = im_gray.data; 
					////std::cout << "3" << std::endl;		
					wholeImage = new ImageRepresentation(pixels, img_size); 

					//std::cout << "Init ADABOOST " << std::endl;
					for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
					{
						printf ("\rinit tracker... %3.0f %% ", ((float)curInitStep)/(iterationInit-1)*100);	
						classifier->update (wholeImage, trackingPatches->getSpecialRect ("UpperLeft"), -1);
						classifier->update (wholeImage, trackedPatch, 1);
						classifier->update (wholeImage, trackingPatches->getSpecialRect ("UpperRight"), -1);
						classifier->update (wholeImage, trackedPatch, 1);
						classifier->update (wholeImage, trackingPatches->getSpecialRect ("LowerLeft"), -1);
						classifier->update (wholeImage, trackedPatch, 1);
						classifier->update (wholeImage, trackingPatches->getSpecialRect ("LowerRight"), -1);
						classifier->update (wholeImage, trackedPatch, 1);
					}
					//std::cout << "Initialized" << std::endl;
					initialized_boosting = true;
				}
				
				//Objavimo detekcije za Kalmana
				////std::cout << "4" << std::endl;
				detectionMsg.detections.push_back(*(best->Create_msg()));
				detectionMsg.header = header;
				detectionMsg.image = *image;
				detections.publish(detectionMsg);				
			}			
			else{
				have_human = false;	
				//std::cout << "Nimamo.: " << std::endl;
		  }			
		}
	}
	/* When the human is detected we classify each found cluster with Online Adaboost */
	else 
	{
		std::cout << "prepoznava" << std::endl;
		if(!(clusters->empty()))
		{
			std::cout << "not" << std::endl;
			int max_size = 0;
			float best_conf = -100.0f;
			float confidence = -100.0f;
			Detector::Rect best_trackedPatch;

			list<Cluster>::iterator largest;
			std::cout << "OK: 1" << std::endl;	
			bool start_img = true;
							
			cv::Mat im_gray;
			std::cout << "OK: 4" << std::endl;	
			cv_bridge::CvImagePtr ib = cv_bridge::toCvCopy(image, image->encoding);
			cv::Mat im_rgb = ib->image;
			cv::cvtColor(im_rgb,im_gray,CV_RGB2GRAY);
			unsigned char *pixels = im_gray.data; 
			wholeImage = new ImageRepresentation(pixels, img_size); 			
			std::cout << "OK: 5" << std::endl;	
			for(it = clusters->begin(); it != clusters->end(); it++ ) 
			{					
				std::cout << "OK: clusters" << std::endl;	
				it->produceRGBBlob(cam_info, image);
				std::cout << "OK: 3" << std::endl;	
				
				//OPTIMIZACIJA: zajememo čb sliko le prvič, saj je pri vseh clustrih enaka.
				trackedPatch = it->getROIofBlob();
			
				Detector::Size trackedPatchSize;
				trackedPatchSize = trackedPatch;
				Patches* patches = new PatchesRegularScan(trackedPatch, validROI, trackedPatchSize, 0.99f);
				detector->classifySmooth (wholeImage, patches);
				std::cout << "Detekcija OK" << std::endl;	
				confidence  = detector->getConfidenceOfBestDetection ();
				std::cout << "Vzami confidence" << std::endl;	
				std::cout << "Primerjamo:" << confidence << std::endl;
				if(confidence > best_conf){
					best_conf = confidence;
					std::cout << confidence << std::endl;
					largest = it;
					best_trackedPatch = trackedPatch;
				}
				delete patches;

			}
			std::cout << "END OF CLUSTERS-------------------" << std::endl;
			//std::cout << "OK - preverimo, ce smo nasli cloveka" << std::endl;
			if (best_conf >= ADABOOST_THRESHOLD){
				//std::cout << "ŠTEVCI OK: Imamo cloveka z ADABOOST: "<< best_conf << std::endl;		
				
				Detector::Size best_trackedPatchSize;
				best_trackedPatchSize = best_trackedPatch;
				/* 
         * Best classified cluster is human. Also we have to update Online Adaboost
         */

				//Visualizacija + klasificiranega ADABOOST
				/*				
				visualization_msgs::Marker mark = largest->Mark_cluster();
				mark.header.stamp = header.stamp;
				mark.header.frame_id = "camera_depth_frame";
				published_marker.publish(mark);*/
				
				patch_left = best_trackedPatch.left;
				patch_upper = best_trackedPatch.upper;
				patch_width = best_trackedPatch.width;
				patch_height = best_trackedPatch.height;
				have_new_patch = true;

				std::cout << "1" << std::endl;
				Detector::Rect trackingROI = largest->getTrackingROI(ADABOOST_SEARCH_AREA, validROI);
				std::cout << "2" << std::endl;
				Patches *trackingPatches = new PatchesRegularScan(trackingROI, validROI, best_trackedPatchSize, 0.99f);
				std::cout << "3" << std::endl;
				classifier->update (wholeImage, trackingPatches->getSpecialRect ("UpperLeft"), -1);
				classifier->update (wholeImage, best_trackedPatch, 1);
				classifier->update (wholeImage, trackingPatches->getSpecialRect ("UpperRight"), -1);
				classifier->update (wholeImage, best_trackedPatch, 1);
				classifier->update (wholeImage, trackingPatches->getSpecialRect ("UpperLeft"), -1);
				classifier->update (wholeImage, best_trackedPatch, 1);
				classifier->update (wholeImage, trackingPatches->getSpecialRect ("LowerRight"), -1);
				classifier->update (wholeImage, best_trackedPatch, 1);

				//delete trackingPatches;
				//frame_counter = 0;
				//prediction_counter = 0;
				std::cout << "4" << std::endl;
				largest->CalculateCentroid();
				largest->ShowBlobInWindow("1");
			
				std::cout << "5" << std::endl;
				detectionMsg.detections.push_back(*(largest->Create_msg()));
				detectionMsg.header = header;
				detectionMsg.image = *image;
				detections.publish(detectionMsg);

				//Detector::prediction::Response pred = getKalmanPrediction();
				
				//calculate_search_window(pred.prediction.x, pred.prediction.y, pred.prediction.z);
				std::cout << "6" << std::endl;
				pcl::getMinMax3D (*(largest->ReturnCluster()), min, max);
				human_min_x = min[0];
				human_max_x = max[0];
				human_min_y = min[1];
				human_max_y = max[1];
				human_min_z = min[2];
				human_max_z = max[2];				
			}
      /* If no cluster is positively classified by Online Adaboost 
       * as human we expand our search window.
       */
			else{	
				//frame_counter++;
				//prediction_counter++;			
				human_min_x -= 0.3;
				human_max_x += 0.3;
				human_min_y -= 0.15;
				human_max_y += 0.15;
				human_min_z -= 0.5;
				human_max_z += 0.3;
				//largest->ShowBlobInWindow("Stevci OK: Ni prepoznave z adaboost");
				//std::cout << "Ni prepoznave clustra z ADABOOST - razsirimo okno iskanja" << std::endl;	
			}
			//delete patches;
		}
		//If no clusters are found
		else{
			//frame_counter++;
			//prediction_counter++;
			human_min_x -= 0.3;
			human_max_x += 0.3;
			human_max_y += 0.15;
			human_min_y -= 0.15;
			human_min_z -= 0.5;
			human_max_z += 0.3;
			std::cout << "Ni clustrov - razsirimo okno iskanja" << std::endl;
			////std::cout << "Vzamemo predikcijo Kalmana. 0.:--------------------------------" << std::endl;
			//Detector::prediction::Response pred = getKalmanPrediction();
			//calculate_search_window(pred.prediction.x, pred.prediction.y, pred.prediction.z);
			//Lahko mogoče še probaš preiskati v okolici Kalmana z Adabootom, če je kje človek
			//CALCULATE NEW SEARCH WINDOWS
	
		}		
		std::cout << "-------------------------------------"<< std::endl;	
	}
}

void Detection_Tracking::calculate_search_window(float x, float y, float z)
{
		//std::cout << "Racunamo okno" << std::endl;
		human_min_x = x - 0.5;
		human_max_x = x + 0.5;
		human_min_y = -2.0;
		human_max_y = 2.0;
		human_min_z = z - 0.5;
		human_max_z = z + 0.5;
}

/*Publish clusters from list<Cluster>*/
void Detection_Tracking::publish_pc_clusters(list<Cluster>* clusters, std::string frame_id, ros::Time stamp, bool show_box){
	list<Cluster>::iterator it;
	pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);		
	//Iterate through all clusters and add them to 1
  for(it = clusters->begin(); it != clusters->end(); it++ ) {
  	*cloud_cluster += (*it->ReturnCluster());

    if(show_box == true){ 
    	visualization_msgs::Marker mark = it->Mark_cluster();
			mark.header.frame_id = frame_id;
    	mark.header.stamp = stamp;
			published_marker.publish(mark);
    }
	}
	cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;	
	
	sensor_msgs::PointCloud2::Ptr clustered_cloud (new sensor_msgs::PointCloud2);
	//convert to ROS type PointCloud2 and add headers
	pcl::toROSMsg (*cloud_cluster, *clustered_cloud);	
	clustered_cloud->header.frame_id = "/camera_depth_frame";  
	clustered_cloud->header.stamp = stamp; 

  //Publish the clusters to ROS
	publish_clusters.publish(*clustered_cloud);
}

/*Publish height in rviz*/
void Detection_Tracking::publish_height (float x, float y, float z, std::string msg, std::string frame_id){
	visualization_msgs::Marker text_obj;
	text_obj.header.frame_id = frame_id;
	text_obj.color.a = 1.0;
	text_obj.color.r = 0.0;
	text_obj.color.g = 1.0;
	text_obj.color.b = 0.0;
	text_obj.scale.x = 0.1;
	text_obj.scale.y = 0.1;
	text_obj.scale.z = 0.1;
	//text_obj.header.stamp = input->header.stamp;
	text_obj.type = 9;
	text_obj.action = 0;
  //text_obj.ns = "KinectDetect";
  text_obj.pose.position.x = x;
  text_obj.pose.position.y = y;
  text_obj.pose.position.z = z;
	//text_obj.lifetime = ros::Duration(0.5);
	
	text_obj.text = msg;
	published_marker.publish(text_obj);			
}


/* Point cloud preprocessing:
 * 	- down-sampling
 *	- seperating down-sampled point cloud to 2 parts (cloud_filtered_a,cloud_filtered_b)
 *	- remove ground from bottom point cloud with RANSAC
 *	- merge both point clouds together
 *  - create clusters and evaluate them if they can be human
 */
void Detection_Tracking::cloud_preprocessing (const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){
	bool data = true;
	//std::cout << "Zacetek procesiranja" << std::endl;
sensor_msgs::PointCloud2::Ptr removedGround (new sensor_msgs::PointCloud2);
	pcl::PointCloud<PointType>::Ptr down_sampled (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr RansacInput (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr input_cloud (new pcl::PointCloud<PointType>);

	//pcl::PointCloud<PointType> copy;
	/*Lower point cloud, where we remove the ground*/
	pcl::PointCloud<PointType>::Ptr cloud_filtered_a (new pcl::PointCloud<PointType>);
 
	/*Upper point cloud that we just combine with the lower one with removed ground*/
	pcl::PointCloud<PointType>::Ptr cloud_filtered_b (new pcl::PointCloud<PointType>);
 
	/*Variables to run ransac from PCL*/
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	
	//pcl::fromROSMsg (*input, *down_sampled);
	pcl::fromROSMsg (*input, *RansacInput);
	
		/*
	 * 1. Start with downsampling point cloud to 
	 *    make further processing faster.
	 * 
	 */
	
	
	pcl::VoxelGrid <PointType> sor;
  sor.setInputCloud (down_sampled);
  sor.setLeafSize (DOWNSAMPLING_LEAF_SIZE, DOWNSAMPLING_LEAF_SIZE, DOWNSAMPLING_LEAF_SIZE);
  sor.filter (*RansacInput);
	 

	/*
	 * 2. Split original point cloud into 2 seperate point clouds.
	 *    Use the lower one in ransac alg. to automaticaly remove 
	 *    ground plane.
	 */

	pcl::PassThrough<PointType> pass (new pcl::PassThrough<PointType>());
  /* If human has been found we search only in the near area of previously detected human and
   * not on whole point cloud.
   */
	if (have_human){
		 std::cout << "Imamo človeka!" << std::endl;
		 pcl::getMinMax3D (*RansacInput, min_kinect_bound, max_kinect_bound);	
		 //Filter X coordinates 
  	 pass.setInputCloud (RansacInput);
		 pass.setFilterFieldName ("x");
		 area_min_x = human_min_x - AREA_X;
		 area_max_x = human_max_x + AREA_X;

		 if(area_min_x < min_kinect_bound[0])
			area_min_x = min_kinect_bound[0];
		 if(area_max_x > max_kinect_bound[0])
			area_max_x = max_kinect_bound[0];
		
		 //std::cout << "Začetek rezanja"<< std::endl;	
     pass.setFilterLimits (area_min_x, area_max_x);
 	   pass.filter (*RansacInput);
		 	
		 //Filter Z coordinates
		 pass.setInputCloud (RansacInput);
		 pass.setFilterFieldName ("z");
		 area_min_z = human_min_z - AREA_Z;
		 area_max_z = human_max_z + AREA_Z;

		 if(area_min_z < min_kinect_bound[2])
			area_min_z = min_kinect_bound[2];
		 if(area_max_z > max_kinect_bound[2])
			area_max_z = max_kinect_bound[2];

     pass.setFilterLimits (area_min_z, area_max_z);
 	   pass.filter (*RansacInput);
		 //std::cout << "Odrezemo"<< std::endl;	
		 pcl::PassThrough<PointType> pass;
		 pass.setInputCloud (RansacInput);

		 /*
		 area_min_y = human_min_y - AREA_Y;
		 area_max_y = human_max_y + AREA_Y;
		 if(area_max_y > max_kinect_bound[1])
			area_max_y = max_kinect_bound[1];
		 if(area_min_y < min_kinect_bound[1])
			area_min_y = min_kinect_bound[1];

		 std::cout << "max_y"<< area_max_y << std::endl;	
		 std::cout << "min_y"<< area_min_y << std::endl;	*/

		 pass.setFilterFieldName ("y");
		 pass.setFilterLimits (0.5, 2.6);
		 pass.filter (*cloud_filtered_a);

			/* Filter the upper one to later merge it with bottom one withouth ground */
		 pass.setFilterLimits (-2.5, 0.5);
		 pass.filter (*cloud_filtered_b);	

		}
  /* If we haven't found or lost human we search the whole area - point cloud */
	else{
			pass = new pcl::PassThrough<PointType>();
		/* Filter the original pc to get bottom one */
			pass.setInputCloud (RansacInput);

			pass.setFilterFieldName ("y");
			pass.setFilterLimits (0.3, 2.6);
		 	pass.filter (*cloud_filtered_a);

			/* Filter the upper one to later merge it with bottom one withouth ground */
			pass.setFilterLimits (-2.5, 0.3);
			pass.filter (*cloud_filtered_b);	
		}
		
		/* 3. Run RANSAC alg. to remove ground from the bottom PC.
		 *    Because we use only the bottom half of original PC, 
		 *    process becomes much faster.
		 */	
		/*Create the segmentation object to segment bottom piece of PC*/
		pcl::SACSegmentation<PointType> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (RANSAC_MAX_ITERATIONS);
		seg.setDistanceThreshold (RANSAC_DISTANCE_THRESHOLD);
		seg.setInputCloud (cloud_filtered_a);

		if(cloud_filtered_a->points.size() > 60){
			//std::cout << "Dovolj tock"<< std::endl;	
			seg.segment (*inliers, *coefficients);
	
			/*Remove the plane from bottom PC*/ 	
			pcl::ExtractIndices<PointType> extract;
			extract.setInputCloud (cloud_filtered_a);
			extract.setIndices (inliers); 
			extract.setNegative (true);
			extract.filter (*input_cloud);
		/* 4. Merge both point clouds together to get 1 point cloud that 
		 *    is used for clustering.
		 */
			*input_cloud += *cloud_filtered_b;
		}
    /*If we don't have enough points we broden our search window*/
		else{
			std::cout << "Premalo tock" << std::endl;
			*cloud_filtered_b += *cloud_filtered_a;
			*input_cloud += *cloud_filtered_b;
			human_min_x -= 0.3;
			human_max_x += 0.3;
			human_max_y += 0.15;
			human_min_y -= 0.15;
			human_min_z -= 0.5;
			human_max_z += 0.3;
			//have_human = false;
			data = false;
		}
  
	if(data){  		
		std::cout << "data" << std::endl;
		/* Creating the KdTree from input point cloud*/
		pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
		tree->setInputCloud (input_cloud);
		/* Here we are creating a vector of PointIndices, which contains the actual index
		 * information in a vector<int>. The indices of each detected cluster are saved here.
		 * Cluster_indices is a vector containing one instance of PointIndices for each detected 
		 * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
		 */
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointType> ec;
		ec.setClusterTolerance (CLUSTER_TOLERANCE); // 12cm
		ec.setMinClusterSize (CLUSTER_MIN_SIZE);	//30
		ec.setMaxClusterSize (CLUSTER_MAX_SIZE); //600
		ec.setSearchMethod (tree);
		ec.setInputCloud (input_cloud);
		/* Extract the clusters out of pc and save indices in cluster_indices.*/
		try{
			ec.extract (cluster_indices);
		}
		catch (exception& e){
			//have_human = false;
			std::cout << "ujel" << std::endl;
			data = false;
		}
		/* To separate each cluster out of the vector<PointIndices> we have to 
		 * iterate through cluster_indices, create a new PointCloud for each 
		 * entry - cluster. Filter them by height and depth. If they pass we 
		 * add them to list of clusters for further processing.
		 */	
		std::vector<pcl::PointIndices>::const_iterator it;
		std::vector<int>::const_iterator pit;
		float height;
		clusters = new list<Cluster>;
		std::cout << "Ok clustering" << std::endl;
		for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {    	
			float max_y = 1000.0f;
			float max_z = -100.0f;
			PointType pt;	
			pcl::PointCloud<PointType>::Ptr cluster_i (new pcl::PointCloud<PointType>);	
		  for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {		
				//Get the highest point of current point cloud
				if(input_cloud->points[*pit].y < max_y){
					max_y = input_cloud->points[*pit].y;
					pt = input_cloud->points[*pit];
				}
				//Get the most far away point of the current point cloud
				 if(input_cloud->points[*pit].z > max_z){
					max_z = input_cloud->points[*pit].z;
				}
				cluster_i -> points.push_back(input_cloud->points[*pit]);  					
		  }
			/*
			sensor_msgs::PointCloud2::Ptr clustered_cloud (new sensor_msgs::PointCloud2);
			pcl::toROSMsg (*cluster_i, *clustered_cloud);	
			clustered_cloud->header.frame_id = "/camera_depth_frame";  
			clustered_cloud->header.stamp = ros::Time::now(); 
			publish_clusters.publish(*clustered_cloud);*/

			std::cout << "naredimo cluster" << std::endl;
			if(!have_human && data){
					std::cout << "visina1" << std::endl;
					height = calculate_height(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3], pt.x, pt.y, pt.z); 
				/*Limit the height and width of a cluster according to human*/
				if ((MIN_ALLOWED_CLUSTER_HEIGHT < height) && (height < MAX_ALLOWED_CLUSTER_HEIGHT) && (max_z < MAX_ALLOWED_CLUSTER_DEPTH) && (max_z > MIN_ALLOWED_CLUSTER_DEPTH)){
					clusters->push_back(*new Cluster(height, cluster_i, max_z, max_y, input->header));
					//publish_height(pt.x, pt.y, pt.z, boost::lexical_cast<std::string>(height), input_cloud->header.frame_id);
				}  
				std::cout << "Ok visina" << std::endl;
			}
		
			else if (data){
					std::cout << "visina2" << std::endl;
				height = calculate_height(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3], pt.x, pt.y, pt.z); 
				std::cout << "Ok visina 2" << std::endl;
				if ((MIN_ALLOWED_CLUSTER_HEIGHT - 0.5 < height) && (height < MAX_ALLOWED_CLUSTER_HEIGHT) && (max_z < MAX_ALLOWED_CLUSTER_DEPTH) && (max_z > MIN_ALLOWED_CLUSTER_DEPTH)){
					clusters->push_back(*new Cluster(height, cluster_i, max_z, max_y, input->header));
					std::cout << "Ok cluster push, visina: "<< height << std::endl;
				}
			}
		
		}	
  if(PUB_BLOBS == true)
		publish_rgb_blobs(clusters, image, cam_info, input->header);
	
  if(PUB_CLUSTERS == true)
		publish_pc_clusters(clusters, input->header.frame_id, input->header.stamp, false);
	
	}
		cv_bridge::CvImagePtr ib = cv_bridge::toCvCopy(image, image->encoding);
		cv::Mat im_rgb = ib->image;
		if(have_new_patch){
			cv::rectangle(im_rgb,
		         Point(patch_left,patch_upper),
		         Point(patch_left+patch_width, patch_upper+patch_height),
		         Scalar( 0, 255, 255 ),
		         3,
		         8 );
			have_new_patch = false;
		}

		cv_bridge::CvImagePtr cv_ptr_image_final(new cv_bridge::CvImage());
		cv_ptr_image_final->header = image->header;
		cv_ptr_image_final->header.frame_id = "/camera_depth_frame";
		cv_ptr_image_final->image = im_rgb;
		cv_ptr_image_final->encoding = "bgr8";
		publish_blob.publish(cv_ptr_image_final->toImageMsg());

		Mark_search_box();
	}

  /*
   * Show the 3D search window.
  */
	void Detection_Tracking::Mark_search_box(){
		Eigen::Vector4f centroid;
		Eigen::Vector4f min;
		Eigen::Vector4f max;
	  visualization_msgs::Marker *searchBox = new visualization_msgs::Marker();
	 
		uint32_t shape = visualization_msgs::Marker::CUBE;
		searchBox->header.frame_id = "/camera_depth_frame";
		searchBox->header.stamp = ros::Time::now();
	 
		//marker.ns = ns;
		searchBox->id = 1;
		searchBox->type = shape;
		searchBox->action = visualization_msgs::Marker::ADD;
	 
		searchBox->pose.position.x = ((area_max_x-area_min_x)/2) + area_min_x;
		searchBox->pose.position.y = ((max_kinect_bound[1]-min_kinect_bound[1])/2) + min_kinect_bound[1];
		searchBox->pose.position.z = ((area_max_z-area_min_z)/2) + area_min_z;
		searchBox->pose.orientation.x = 0.0;
		searchBox->pose.orientation.y = 0.0;
		searchBox->pose.orientation.z = 0.0;
		searchBox->pose.orientation.w = 1.0;
	 
		searchBox->scale.x = (area_max_x-area_min_x);
		searchBox->scale.y = (max_kinect_bound[1]-min_kinect_bound[1]);
		searchBox->scale.z = (area_max_z-area_min_z);
	 
		if (searchBox->scale.x ==0)
		    searchBox->scale.x=0.1;

		if (searchBox->scale.y ==0)
		  searchBox->scale.y=0.1;

		if (searchBox->scale.z ==0)
		  searchBox->scale.z=0.1;
		 
		searchBox->color.r = 100;
		searchBox->color.g = 49;
		searchBox->color.b = 237;
		searchBox->color.a = 0.5;

		//marker.lifetime = ros::Duration();
		searchBox->lifetime = ros::Duration(0.5);
		published_marker.publish(*searchBox);
} 
  Detection_Tracking::~Detection_Tracking(){
		delete track_mark;
		delete clusters;
		delete wholeImage;
	}
}
