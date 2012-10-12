#include <ros/ros.h>
#include "kalmanStates.cpp"
#include <Detector/Detection.h>
#include <boost/thread.hpp>
#include <math.h>

int HUMAN_THRESHOLD = 0;
//How many Kalman steps to be made when lost detection
int FRAMES_TO_VALIDATE_THRESHOLD = 30;
//How many frames withouth detection for a track to get deleted
int FRAMES_OLD_THRESHOLD = 200;
boost::mutex m_mutex; 
namespace Tracker{
	//Normal PointStamped point position with variable if position created by Kalman alg. or not.
	class GoalPoint{
		private:
			bool is_kalman;

		public:
			geometry_msgs::PointStamped position;		
			bool is_kalman_point(){
				return is_kalman;
			}

			GoalPoint(geometry_msgs::PointStamped pos, bool kalman){
				position = pos;
				is_kalman = kalman;
			}	
	};

	class Position2d{
		public:
			 float x;
			 float y;

			 Position2d(float x1,float y1){
				x = x1; y = y1;		
			 };
	};

	class Track{
		protected:
			bool tracking;
		  bool is_new;
			bool is_human;
			bool visibility;
			bool new_data;
			int classifier; //TODO class classifier
			int is_human_count;
			int is_missing_count;
			int frames_from_last_human_detection;
			int frames_old;
			Detector::Detection detection;
			MahalanobisParameters* mp;
		  KalmanStates* kalman;
			geometry_msgs::PointStamped previous_position;
				
		public:	  	
			int r;int g;int b;
			std::list<GoalPoint>* track_points;

		  Track(KalmanStates* kal, Detector::Detection det, geometry_msgs::PointStampedPtr& pos):
				kalman(kal), 
				r(std::rand() % 255),
				g(std::rand() % 255),
				b(std::rand() % 255),
				is_new(true),
				new_data(false),
				is_human(true),
				tracking(false),
				is_human_count(1),
				is_missing_count(0),
				frames_from_last_human_detection(0),
				frames_old(0),
				detection(det),
				track_points(new std::list<GoalPoint>), 
				mp(new MahalanobisParameters())
		  	{
				GoalPoint *p = new GoalPoint(*pos, false);
				track_points->push_back(*p);
				};
		  ~Track(){
				if(kalman)
					delete kalman;
				if(mp)
					delete mp;
				if(track_points)
					delete track_points;
			};
			//Tocko dodamo v mapo, le ce je tocka od prejsnje oddaljena 0.5m
			void add_position(geometry_msgs::PointStamped pos, float is_kalman){
				boost::mutex::scoped_lock l(m_mutex);
				if(calc_distance(pos.point.x, pos.point.y, previous_position.point.x, previous_position.point.y) > 0.25){
					//Če je zadnja točka, točka detekcije, izbriši iz liste ven vse Kalmanove točke.
					if(!is_kalman)
						clear_kalman_points();

					GoalPoint *p = new GoalPoint(pos, is_kalman);
					track_points->push_back(*p);
					previous_position = pos;
				}
			};

			void clear_kalman_points(){
				std::list<GoalPoint>::iterator it;
				for(it = track_points->begin(); it != track_points->end(); it++ ) {	
					if(it->is_kalman_point())
						it = track_points->erase(it);
				}
			}

			float calc_distance(float x1, float y1, float x2, float y2){
				return sqrt(pow(double(x2 - x1),2.0) + pow(double(y2 - y1),2.0));
			}
                        
      std::list<GoalPoint>* get_track_points(){
				boost::mutex::scoped_lock l(m_mutex);
				return track_points;
			}
			geometry_msgs::PointStamped get_latest_position(){
				boost::mutex::scoped_lock l(m_mutex);
				geometry_msgs::PointStamped f = (track_points->back()).position;
				track_points->pop_back();
				return f;
			}

			geometry_msgs::PointStamped get_first_position(){
				boost::mutex::scoped_lock l(m_mutex);
				geometry_msgs::PointStamped f = (track_points->front()).position;
				track_points->pop_front();
				return f;
			}

			geometry_msgs::PointStamped get_first_position_withouth_remove(){
				return (track_points->front()).position;
			}

			bool has_new_data(){
				boost::mutex::scoped_lock l(m_mutex);
				return new_data;
			};

			void set_new_data(bool dat){
				boost::mutex::scoped_lock l(m_mutex);
				new_data = dat;
			};
			bool is_tracking(){
				return tracking;
			}
			void set_tracking(bool tr){
				tracking = tr;
			}
			bool check_is_new(){
				return is_new;
			};
			void set_new(bool newTrack){
				is_new = newTrack;
			};
			bool check_if_human(){
				return is_human;
			};		
			void set_if_human(bool human){
				is_human = human;
			};
			void set_kalman(KalmanStates* kal){
				boost::mutex::scoped_lock l(m_mutex);
				kalman = kal;
			};
			KalmanStates* get_kalman(){
				boost::mutex::scoped_lock l(m_mutex);
				return kalman;
			};  
		
			bool is_old(){
				if(frames_old > FRAMES_OLD_THRESHOLD)
					return true;

				else
					return false;
			};
			void set_detection(Detector::Detection det){
				boost::mutex::scoped_lock l(m_mutex);
				detection = det;
				
				new_data = true;	
				is_human_count++;

				if (is_human_count > HUMAN_THRESHOLD && frames_from_last_human_detection < FRAMES_TO_VALIDATE_THRESHOLD){
					is_human = true;
					//std::cout << frames_from_last_human_detection << " Jejj imamo človeka" << std::endl;
				}
				frames_from_last_human_detection = 0;
				frames_old = 0;
				is_missing_count = 0;	
			};

			Detector::Detection get_detection(){
				boost::mutex::scoped_lock l(m_mutex);
				return detection;
			};

			void set_malanobi(){
				if (kalman){
					boost::mutex::scoped_lock l(m_mutex);
					kalman->getMahalanobisParameters(*mp);
				}
			};
			void add_1_to_frames_counter(){
				frames_from_last_human_detection++;
				frames_old++;
				if (frames_from_last_human_detection > FRAMES_TO_VALIDATE_THRESHOLD){			
					is_human = false;
					//std::cout <<frames_from_last_human_detection << " Ni vec cloveka." << std::endl;
					frames_from_last_human_detection = 0;
					is_human_count = 0;
				}
			}

			const MahalanobisParameters get_malanobi(){
				return *mp;
			};	
	};
}
