#ifndef SYSTEM_H_
#define SYSTEM_H_



#include "ORBVocabulary.h"
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;

#include <mutex>
#include <thread>
#include <iomanip>
#include "KeyFrame.h"
#include "Converter.h"
#include "configparam.h"

namespace vill {

    class Viewer;

    class FrameDrawer;

    class Map;

    class MapPoint;

    class Tracking;

    class LocalMapping;

    class LoopClosing;

    class MapDrawer;

    class KeyFrameDatabase;

    class System {
    public:
	static void SaveCurrentKF(KeyFrame* pKF0){
	    static bool fopened = false;
	    static ofstream fckf;
	    if (!fopened)
	    {
					// Need to modify this to correct path
		string tmpfilepath = ConfigParam::getTmpFilePath();
		fckf.open(tmpfilepath + "CurrentKFTraj.txt");
		if (fckf.is_open())
			fopened = true;
		else{
			cerr << "file open error in SaveCurrentKF" << endl;
			fopened = false;
		}
		fckf << std::fixed << std::setprecision(6);
	    }

	    cv::Mat R = Converter::toCvMat(pKF0->GetRotation()).t();
	    vector<float> q = Converter::toQuaternion(R);
	    Vector3f t = pKF0->GetCameraCenter();
	    
	    fckf << setprecision(6) << pKF0->mTimeStamp << setprecision(7) << " " << t[0] << " " << t[1] << " " << t[2]
		  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
	}
      
   
	void SaveLocationResult(const string &filename);
      
    public:

        // Input sensor
        enum eSensor {
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2
        };

    public:

        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System(const string &strVocFile, const string &strSettingsFile/*, const string &strLaserMap*/, const eSensor sensor,
               const bool bUseViewer = true, ConfigParam *pParams = NULL);
		
//         System(const string &strVocFile, const string &strSettingsFile, const string &strLaserMap, const eSensor sensor,
//                const bool bUseViewer = true, ConfigParam *pParams = NULL);
		
		// Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
// 		System(const eSensor sensor,const string &strVocFile, const string &strSettingsFile, const string &strDataFile,
// 			   const string &strLaserMap,
// 			 const bool bUseViewer = true);
		


		//construct function with loop closure
		System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                bool bUseViewer,  bool openLoop , ConfigParam *pParams = NULL);
		

        ~System();

        // Proccess the given stereo frame. Images must be synchronized and rectified.
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

		
		void SaveToTXT(const string &filename);
			



        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();

        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear map)
        void Reset();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);
		
		void SaveLoops(const string &filename);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);
		void SaveTrajectoryKITTIwithTime(const string &filename);

        // save keyframe trajectory
        void SaveKeyFrameTrajectoryNavState(const string &filename);

        // TODO: Save/Load functions
        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();

        std::vector<MapPoint *> GetTrackedMapPoints();

        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        // check if local map can accept keyframes (not busy)
        bool GetLocalMapAcceptKF();
		
		
		
		void saveData(const string& filepath);
		void saveDataKFMP(const string& filepath);
		void saveDepthMap(const string& filepath);
		
		
		
		bool loadData(const string& strDataFile);
		bool loadData(const string& strDataFile, bool load_line, bool load_mapimg);
		
		void setmapimgPath(const string& filepath){mapimgPath=filepath;}

        // Proccess the given monocular frame with IMU measurements
        // IMU measurements should be between LAST and CURRENT frame
        // (i.e. the timestamp of IMU measrements should be smaller than the timestamp of image)
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).

        ConfigParam *mpParams = nullptr;  // IMU related params
		ofstream fconfigSystem;
		
		LocalMapping* getLocalMappingPtr();
		Tracking* getTrackingPtr();
		
		std::mutex mMutexTrackingPoseUpdated;
		bool trackingPoseUpdated;
		SE3f currentPoseTcw;
		double currentTime;
		bool GetTrackingPoseUpdated(SE3f& pose,  double& cTime );
		void SetTrackingPoseUpdated(SE3f& currentpose, const double& currentTime);
		
		Map* getMapPtr(){return mpMap;}
		
		bool CheckLocalMappingState();
		
		std::map<int, cv::Mat> getDepthMap();
		void update_gradient(map<int, cv::Mat>& m_id_gradient, map<int, float>& m_id_loss);
	
	
    private:

        // Input sensor
        eSensor mSensor;

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary =nullptr;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase =nullptr;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map *mpMap =nullptr;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking *mpTracker =nullptr;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping *mpLocalMapper =nullptr;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        LoopClosing *mpLoopCloser =nullptr;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer *mpViewer =nullptr;

        FrameDrawer *mpFrameDrawer =nullptr;
        MapDrawer *mpMapDrawer =nullptr;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread *mptLocalMapping =nullptr;
        std::thread *mptLoopClosing =nullptr;
        std::thread *mptViewer =nullptr;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset =false;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPoints;
        std::mutex mMutexState;
		
		Matrix4f mTvelo_cam_zed_left, mTvelo_cam_inv_right;
		
		// load map img path
		string mapimgPath;
    };

}// namespace vill

#endif // SYSTEM_H
