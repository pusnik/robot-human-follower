#include "tracking.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(Tracker, Tracking, Tracker::Tracking, nodelet::Nodelet);

namespace Tracker{
	void Tracking::onInit(){ 
			nh.reset(new ros::NodeHandle(getNodeHandle()));
			mTransformListener = new tf::TransformListener(ros::Duration(20.0), true);
			tracks = new std::list<Track>;

			track_mark = new visualization_msgs::Marker(); 		

			dt = 0.083333333;			
			
			detect.subscribe(*nh, "detections", 1);
			odom.subscribe(*nh, "odom", 1);
			sync.reset(new Synchronizer(MySyncPolicy(32), detect, odom));
			sync->registerCallback(boost::bind(&Tracking::start_tracking, this, _1, _2));

			published_marker = nh->advertise<visualization_msgs::Marker> ("markers",0);
			cmdpub = nh->advertise<geometry_msgs::Twist> ("cmd_vel", 1);

			ac = new MoveBaseClient("move_base", true);

			running = true;
			kalmanThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Tracking::KalmanLoop, this)));
			driverThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Tracking::DriverLoop, this)));

			
			predictions = nh->advertiseService("prediction", &Tracking::get_prediction, this);
	}

	Tracking::~Tracking(){
    		if (running){
					//cmdpub.publish(geometry_msgs::Twist());
        	NODELET_INFO("shutting down Kalman thread");
					NODELET_INFO("shutting down Driver thread");
          running = false;
          kalmanThread->join();
					driverThread->join();
        	NODELET_INFO("Kalman thread stopped");
					NODELET_INFO("Driver thread stopped");
      	}
				if(tracks)
					delete tracks;	
				if(mTransformListener)
					delete mTransformListener;
				if(ac)
					delete ac;
				if(track_mark)
					delete track_mark;
  }

	bool Tracking::get_prediction(Detector::prediction::Request& request, Detector::prediction::Response& response)
	{
		ROS_INFO("Requesting Kalman prediction.");
		if (!latest_prediction)
			{
				  ROS_INFO("We don't have Kalman prediction yet.");
				  return false;
			}

		response.prediction.x = latest_prediction->point.x;
		response.prediction.y = latest_prediction->point.y;
		response.prediction.z = latest_prediction->point.z;

		return true;

	}

	void Tracking::start_tracking (const Detector::DetectionListConstPtr& detects, const nav_msgs::OdometryConstPtr& odom){
		//Convert observed kinect coordinates to world using tf library
		////std::cout << "DETECTIONS: " << std::endl;
		double GATE = 0.999;
		double M = 100.0;
		//position is different in robo coordinates
		int nrows = tracks->size();
		int ncols = detects->detections.size();
		munkres_matrix::Matrix<double> cost_matrix(nrows, ncols);

		//If there is no tracks make optimization and automacially create new track for every detection
		if(nrows > 0){
				//to all tracks add +1 to frames_from_last_human_detection
				//If track has no human detected for more than X frames then
				//track is no longer validated as human
				//Get all detections from this frame and check what
				//tracks can we associate to them. 
				//First calculate Mahalanobi distance for det i from track j
				//In the end get cost_matrix and solve it with Munkres alg.
				////std::cout << "			- Loop through all detections:" << std::endl;

				for(int i=0; i < detects->detections.size(); i++){
					std::list<Track>::iterator it;
					geometry_msgs::PointStampedPtr p (new geometry_msgs::PointStamped);
					geometry_msgs::PointStampedPtr worldPoint (new geometry_msgs::PointStamped);
					p->header.frame_id = detects->header.frame_id;
					p->header.stamp = detects->header.stamp;				     
          p->point.x = detects->detections[i].centroid.x;
          p->point.y = detects->detections[i].centroid.y;
          p->point.z = detects->detections[i].centroid.z;
					
					*worldPoint = getWorldPoint(p, "/odom");
					
					int j = 0;
					////std::cout << "								- loop through tracks" << std::endl;
					for(it = tracks->begin(); it != tracks->end(); it++ ) {
						it->set_malanobi();
						double distance = it->get_kalman()->performMahalanobisDistance(worldPoint->point.x, worldPoint->point.y, it->get_malanobi());
						if(distance < GATE)
							cost_matrix(j,i) = distance * distance;
						else
							cost_matrix(j,i) = M;
						j++;
					}
				}
				// Apply Munkres algorithm to matrix.
				////std::cout << "				 - Apply munkres algorithem" << std::endl;
				Munkres m;
				m.solve(cost_matrix);

				//Check if we have any tracks that correspond to new detections
				////std::cout << "				 - Check for correspondings" << std::endl;
				geometry_msgs::PointStampedPtr p(new geometry_msgs::PointStamped);
				geometry_msgs::PointStampedPtr worldPoint(new geometry_msgs::PointStamped);
				for (int j = 0 ; j < ncols; j++){
					int have_track = 0;
					std::list<Track>::iterator it;	
					int i = 0;					
					for(it = tracks->begin(); it != tracks->end(); it++ ) {
						if (cost_matrix(i,j) == 0 ){ //Imamo ustrezen track, ki pripada te detekciji
							////std::cout << "					- Detection's corresponding track found." << std::endl;					
							p->header.frame_id = detects->header.frame_id;
							p->header.stamp = detects->header.stamp;		
							p->point.x = detects->detections[j].centroid.x;
						  p->point.y = detects->detections[j].centroid.y;
						  p->point.z = detects->detections[j].centroid.z;
							*worldPoint = getWorldPoint(p, "/odom");
							it->set_detection(detects->detections[j]);
						  it->set_tracking(true);
							have_track = 1;						
							break;
						}
					i++;
					}	
					if (have_track = 0){
						//std::cout << "NOV TRACK--------------------------------------" << std::endl;					
						p->header.frame_id = detects->header.frame_id;
					  p->header.stamp = detects->header.stamp;		
						p->point.x = detects->detections[j].centroid.x;
						p->point.y = detects->detections[j].centroid.y;
						p->point.z = detects->detections[j].centroid.z;
					
						*worldPoint = getWorldPoint(p, "odom");
						create_new_track(detects->detections[j],
															worldPoint, 
															detects->detections[j].distance);
					}
				}
		}
		//If no tracks, create new track for every detection
		else{
			////std::cout << "				NO tracks. Creating new tracks for each detection" << std::endl;
			for(int i=0; i < detects->detections.size(); i++){
			  geometry_msgs::PointStampedPtr p (new geometry_msgs::PointStamped);
				geometry_msgs::PointStampedPtr worldPoint (new geometry_msgs::PointStamped);
				p->header.frame_id = detects->header.frame_id;
				p->header.stamp = detects->header.stamp;		
				p->point.x = detects->detections[i].centroid.x;
        p->point.y = detects->detections[i].centroid.y;
        p->point.z = detects->detections[i].centroid.z;
				
				*worldPoint = getWorldPoint(p,"odom");
				create_new_track(detects->detections[i], 
												  worldPoint, 
													detects->detections[i].distance);
			}
		}
	}

	/* Function for driving the robotic system to certain points (human) in the world. It runs in its own thread. */
	void Tracking::DriverLoop(){
			geometry_msgs::PointStampedPtr worldPoint (new geometry_msgs::PointStamped);
			std::list<Track>::iterator it;
			bool start_sleep = true;
			ros::Rate r(5);

			while(running)
			{
				for(it = tracks->begin(); it != tracks->end(); it++ )
				{	
					if(start_sleep == true){
						start_sleep = false;
						ros::Duration(1.5).sleep();
					}
			
					if( (it->get_track_points())->size() > 0)
					{
						*worldPoint = it->get_first_position();

						float x = worldPoint->point.x;
						float y = worldPoint->point.y;

						//Absolute angle must be used for correct robot orientation towards goal	
						goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (std::atan2(y,x));
						goal.target_pose.header.frame_id = "/odom";
					  goal.target_pose.header.stamp = ros::Time::now();
					  goal.target_pose.pose.position.x = x;
					  goal.target_pose.pose.position.y = y;				
						

					  ROS_INFO("Sending goal");						
					  ac->sendGoal(goal);					
					  //ac->waitForResult();				
					}
				}
			r.sleep();
			}
	}

  /* Function for getting Kalman predictions. It runs in its own thread at 12 fps. 
   * If the node for detection detects human it sends the point to the Kalman filter.
   * Kalman filter corrects the position and saves it to list od positions.
   * If no human detection is found at current frame, Kalman filter is used for predicting human position,
   * according to previous movement.
   */
	void Tracking::KalmanLoop(){
			std::list<Track>::iterator it;
			double *x;		double *y;
			bool is_kalman_point = false;
			//std::cout << "KALMAN LOOP:" << std::endl;
			ros::Rate r(12);
			//boost::mutex m_mutex; 
			geometry_msgs::PointStampedPtr p2(new geometry_msgs::PointStamped);
			geometry_msgs::PointStampedPtr worldPoint2(new geometry_msgs::PointStamped);
			geometry_msgs::PointStampedPtr kalmanPoint(new geometry_msgs::PointStamped);
			
			while(running){
				//Check for old tracks and delete them if needed
				/*
				for(it = tracks->begin(); it != tracks->end(); it++ ) {
					if(it->is_old()){				
						//std::cout << "Star track - brisemo"<< std::endl;
						it = tracks->erase(it);
					}
				}
				*/
				for(it = tracks->begin(); it != tracks->end(); it++ ) {	
					it->add_1_to_frames_counter();	
					if(it->check_if_human()){
						x = new double();
						y = new double();
						//vx = new double();
						//vy = new double();
				
						////std::cout << "		-predict" << std::endl;
						it->get_kalman()->predict();
						it->get_kalman()->update();
					
							// IF new data Make an observation
						if(it->has_new_data()){
							is_kalman_point = false;
							//std::cout << "		-new data received, conf: "<< it->get_detection().confidence << std::endl;
							it->get_kalman()->predict();	
							p2->header.frame_id = it->get_detection().header.frame_id;
							p2->header.stamp = it->get_detection().header.stamp;	
							p2->point.x = it->get_detection().centroid.x;
         		  p2->point.y = it->get_detection().centroid.y;
          	  p2->point.z = it->get_detection().centroid.z;
						  *worldPoint2 = getWorldPoint(p2, "/odom");

							it->get_kalman()->update(worldPoint2->point.x, 
																			 worldPoint2->point.y, 
																			 it->get_detection().distance);

							it->set_new_data(false);
						}
						//Ce nimamo detekcije samo dopolnemo potrebne informacije za tocke
						else{
							is_kalman_point = true;
							worldPoint2->header.frame_id = "/odom";
							worldPoint2->header.stamp = ros::Time::now();	
          	  worldPoint2->point.z = 0.0;
							
						}
						it->get_kalman()->getState(*x,*y);
						Mark_path(*x,
											*y, 
											it->get_detection().distance, 
											it->r, it->g, it->b);
						
						worldPoint2->point.x = *x;
						worldPoint2->point.y = *y;	
					  it->add_position(*worldPoint2, is_kalman_point);	
						
						/*Mark_path(it->get_first_position_withouth_remove().point.x,
											it->get_first_position_withouth_remove().point.y, 
											0.0, 
											it->r, it->g, it->b);*/
							 
					}
				}		
				r.sleep();
			}
	}

  /* Publish markers */
	void Tracking::Mark_path(double x, double y, double z, int r, int g, int b){ 
		uint32_t shape = visualization_msgs::Marker::CYLINDER;
		 

		track_mark->id = rand() % 1000000000 + 1;
		track_mark->type = shape;
		track_mark->action = visualization_msgs::Marker::ADD;
	 
		track_mark->pose.position.x = x;
		track_mark->pose.position.y = y;
		//track_mark->pose.position.z = z;
		track_mark->pose.orientation.x = 0.0;
		track_mark->pose.orientation.y = 0.0;
		track_mark->pose.orientation.z = 0.0;
		track_mark->pose.orientation.w = 1.0;
	 
		track_mark->scale.x = 0.08;
		track_mark->scale.y = 0.08;
		track_mark->scale.z = 0.02;
		   
		track_mark->color.r = r;
		track_mark->color.g = g;
		track_mark->color.b = b;
		track_mark->color.a = 0.5;

		//marker.lifetime = ros::Duration();

		track_mark->header.frame_id = "/odom";
		track_mark->header.stamp = ros::Time();

		published_marker.publish(*track_mark);
	} 

	void Tracking::create_new_track(const Detector::Detection detection, geometry_msgs::PointStampedPtr& worldPoint, double z){
			//std::cout << "Init kalman" << std::endl;
			//std::cout << "Data: x:"<< worldPoint->point.x << " y:" << worldPoint->point.y << std::endl;
		  ////std::cout << "x:2.0 y:1.8 d:2.5" << std::endl;
			KalmanStates *tracker = new KalmanStates(dt, 0.003 ,4.0);
			tracker->init(worldPoint->point.x, worldPoint->point.y, z);
			Track *t = new Track(tracker, detection, worldPoint);
			//Mark_path(worldPoint->point.x, worldPoint->point.y, z, t->r, t->g, t->b);
			//make_command(worldPoint);

			tracks->push_back(*t); 
	}

	void Tracking::make_command(geometry_msgs::PointStampedPtr& worldPoint){
		/*
		geometry_msgs::Twist cmd;
		geometry_msgs::Twist cmd2;
		geometry_msgs::PointStamped roboPoint;
		roboPoint = getWorldPoint(worldPoint, "/base_link");
		////std::cout << "Premik na x:"<< worldPoint->point.x << " y:" << worldPoint->point.y << std::endl;
		
    cmd.angular.z = (roboPoint.point.y) * 0.2;    

    float position_x = (roboPoint.point.x - 2.1);
    if (position_x < 0){
      cmd.linear.x = position_x * 0.3;
    }
    else{
      cmd.linear.x = position_x * 0.2;
     
   	}
   
    cmdpub.publish(cmd);
		*/
	}

	geometry_msgs::PointStamped Tracking::getWorldPoint(geometry_msgs::PointStampedPtr& p, const char *frame){
		geometry_msgs::PointStamped result;
    try{
			mTransformListener->waitForTransform(p->header.frame_id, frame, p->header.stamp, ros::Duration(5.0));
      mTransformListener->transformPoint(frame, *p, result);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
			//std::cout << "Error" << std::endl;
      return geometry_msgs::PointStamped();
    }

    return result;
	}

}
