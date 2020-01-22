#ifndef MESSAGEVIEWER_H_
#define MESSAGEVIEWER_H_

#include <mutex>
#include <thread>
#include <string>
#include "ros/ros.h"
#include "ros/publisher.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
// #include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/tf.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "System.h"
#include "LocalMapping.h"
#include "glog/logging.h"


#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "pointmatcher/PointMatcher.h"

using namespace std;

namespace vill{
  class System;
  class LocalMapping;
  class Tracking;
  
  class MessageViewer{
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
  public:
    MessageViewer(ros::NodeHandle& n, vill::System* System, ros::Rate& r_);
    
    void InsertMessage();
    
    void publishMessage();
    
    void Loop();
	
	void setTbc(const Eigen::Matrix4d& Tbc_);
	
	void setDoLocalization(bool flag);
    
  private:
    System* mSystem;
    LocalMapping * mLocalMapping;
	Tracking* mTracker;
    ros::NodeHandle& nh;
    
    ros::Publisher velocityPub;
    ros::Publisher biasaPub;
    ros::Publisher biasgPub;
    ros::Publisher gravityPub;
    ros::Publisher thomas_velocityVector3Pub;
	ros::Publisher poseStampedPub, poseStampedPub_msf;
	ros::Publisher kfPub;
	ros::Publisher mpPub;
    ros::Subscriber thomas_velocity;
	ros::Subscriber localization_state;
	ros::Subscriber opt_mappoints_sub;
	ros::Subscriber opt_localkeyframes_sub;
	
	tf::TransformBroadcaster tfBroadcaster;
    
    ros::Rate r;
    
	std::mutex mMutexStateUpdate;
	
	vill_odom_nodelet::LocalKeyframesPtr keyframes_for_localization;
	vill_odom_nodelet::LocalMappointsPtr mappoints_for_localization;
	
    void listenVel(const geometry_msgs::Twist& VelocityMsgIn);
	void listen_localization_state( std_msgs::Int8ConstPtr state ); //working - don't have to save; 0
																	//done&OK - save current sliding window 1
																	//done&false - save longer sliding window 2
																	//too long vacancy 3
	void listen_updated_mappoints(const sensor_msgs::PointCloud2& opt_mappoints_msg);
	void listen_updated_localkeyframes(const vill_odom_nodelet::opt_keyframes& opt_kfs_);
    void publish_states_for_localization();
	
	Eigen::Matrix4d Tcb;
	
	double last_pub_time;
	
	bool doLocalization;
	
	double cnt_localization_delta;
	
  };
}


#endif //Message_Viewer