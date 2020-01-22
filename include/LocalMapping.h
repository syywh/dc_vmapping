#ifndef LOCALMAPPING_H_
#define	LOCALMAPPING_H_


#include "Viewer.h"
#include "System.h"
#include "Tracking.h"
#include "LoopClosing.h"
/*
#include "vill_odom_nodelet/MapPoint.h"
#include "vill_odom_nodelet/KeyFrame.h"

#include "vill_odom_nodelet/LocalKeyframes.h"
#include "vill_odom_nodelet/LocalMappoints.h"
#include "vill_odom_nodelet/KeyFrame_marginal.h"
#include "vill_odom_nodelet/pose_edge.h"*/

#include "pointmatcher/PointMatcher.h"

#include <Eigen/Core>
using namespace Eigen;

using namespace Sophus;

#include <mutex>
#include <list>
#include <vector>
#include <thread>
#include <fstream>

using namespace std;

// #include "LaserMap.h"
//#include <ros/builtin_message_traits.h>


namespace vill {

class Tracking;

class Map;

class KeyFrame;

class KeyFrameDatabase;

class ConfigParam;

class MapPoint;

class Viewer;

//Inner class of Keyframe used in Visual-Inertial initialization
// class KeyFrameInit {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 
//     double mTimeStamp;
//     KeyFrameInit *mpPrevKeyFrame;
//     SE3d Twc;
//     IMUPreintegrator mIMUPreInt;
//     std::vector<IMUData> mvIMUData;
//     Vector3d bg;
// 
//     KeyFrameInit(KeyFrame &kf);
//     void ComputePreInt(void);
// };

class LocalMapping {
public:
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

    LocalMapping(Map *pMap, const float bMonocular/*, LaserMap* pLaserMap*/, ConfigParam *pParams);
// 	LocalMapping(Map *pMap, const float bMonocular, LaserMap* pLaserMap, ConfigParam *pParams);

    void SetTracker(Tracking *pTracker);
	
	void SetLoopCloser(LoopClosing* pLoopClosure);

    // Main function
    void Run();
	
	// Main function
    void RunOffline();

    void InsertKeyFrame(KeyFrame *pKF);
    
    // Thread Synch
    void RequestStop();

    void RequestReset();

    bool Stop();

    void Release();

    bool isStopped();

    bool stopRequested();

    bool AcceptKeyFrames();

    void SetAcceptKeyFrames(bool flag);

    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();

    bool isFinished();

    int KeyframesInQueue() {
	unique_lock<std::mutex> lock(mMutexNewKFs);
	return mlNewKeyFrames.size();
    }
    
    // verbose flag
    int mverbose;
    
    //Config parameters
    ConfigParam *mpParams= nullptr;
    
    bool mbUseIMU;
	
	bool mbUseLaserpose;
	
	bool mbUseLaserMap;
    
    std::map<double, std::pair<vector<float>, Vector3f>> mPoses;

    //Thread for Visual-Inertial alignment
    std::thread *mptLocalMappingVIOInit =nullptr;

    // KeyFrames in Local Window, for Local BA
    // Insert in ProcessNewKeyFrame()
    void AddToLocalWindow(KeyFrame *pKF);

    void DeleteBadInLocalWindow(void);

    std::mutex mMutexVINSIniting;
    bool mbVINSIniting;

    bool GetVINSIniting(void);

    void SetVINSIniting(bool flag);

    bool mbResetVINSInit;

    bool GetResetVINSInit(void);

    bool SetResetVINSInit(bool flag);

    void VINSInitThread(void);

    bool TryInitVIO(void);
	    
    bool GetVINSInited(void);

    void SetVINSInited(bool flag);

    bool GetFirstVINSInited(void);

    void SetFirstVINSInited(bool flag);

    void setGravityVec(Vector3d &gw);
    
    Vector3d GetGravityVec(void);

    double GetVINSInitScale(void) { return mnVINSInitScale; }

    bool GetMapUpdateFlagForTracking();

    void SetMapUpdateFlagInTracking(bool bflag);

    KeyFrame *GetMapUpdateKF();

    std::mutex mMutexUpdatingInitPoses;

    bool GetUpdatingInitPoses(void);

    void SetUpdatingInitPoses(bool flag);
    
    bool isMessageUpdate();
    void SetMessageUpdate();
    void ResetMessageUpdate();
    Vector3d getVelocity();
    Vector3d getBiasa();
    Vector3d getBiasg();
    Vector3d getGravity();
// 	void getLocalizationMessage(vill_odom_nodelet::LocalKeyframesPtr& kf_for_localization, vill_odom_nodelet::LocalMappointsPtr& mp_for_localization );
// 	void setLocalizationStateUpdate(const int& state );
// 	void saveStatesforLocalization(list<MapPoint*>& lMapPoints, list<KeyFrame*>& lFixedKeyFrames);
// 	void saveStatesforLocalization_step1(list<MapPoint*>& lMapPoints, const list<KeyFrame*>& llocalKeyFrames, Eigen::MatrixXd& new_info);
// 	void saveStatesforLocalization_step2();
	void computeJacobs(Matrix3d& dRij_T, Matrix3d& RiT);
	void computeEdges();
	
	void setboolMapPointsUpdatedByLocalization();
	bool getboolMapPointsUpdatedByLocalization();
	void setvectorUpdatedMappointsbyLocalization(DP& opt_mappoints_);
	void getvectorUpdatedMappointsbyLocalization(DP*& opt_mappoints_);
	
	void setboolLocalKeyframeUpdatedByLocalization();
	bool getboolLocalKeyframeUpdatedByLocalization();
	void setvectorUpdatedLocalKeyFramesByLocalization(vector<Matrix4d>& opt_kfs_, vector<int>& opt_ids_);
	void getvectorUpdatedLocalKeyFramesbyLocalization(vector<Matrix4d>& opt_kfs_, vector<int>& opt_ids_);
	
	SE3f getInitialTWi();
	
	

protected:

    // visual global BA
    void GlobalBA(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
		    const unsigned long nLoopKF = 0, const bool bRobust = true);
    
    // VI global BA
    void GlobalBANavState(Map *pMap, const Vector3d &gw, int nIterations, bool *pbStopFlag,
		    const unsigned long nLoopKF, const bool bRobust);
    
    void LocalBA(void);
	
	void LocalBA_imu_LaserMap(void);
	
	void LocalBA_imu_LaserPose(void);
	
	void LocalBA_imu(void);
	
	void LocalBA_onlyVision(void);
    
//     Vector3d OptInitGyroBias(const vector<SE3d> &vTwc, const vector<IMUPreintegrator> &vImuPreInt);
	
// 	void OptInitPitchRollBa(vector<IMUData>& vIMUdata, Vector2d& pitch_roll, Vector3d& biasa);
    
    bool CheckNewKeyFrames();

    void ProcessNewKeyFrame();

    void CreateNewMapPoints();

    void MapPointCulling();

    void SearchInNeighbors();

    void KeyFrameCulling();

    Matrix3f ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

    bool mbMonocular;

    void ResetIfRequested();

    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();

    void SetFinish();

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map *mpMap =nullptr;
//     LaserMap* mpLaserMap = nullptr;

    Tracking *mpTracker =nullptr;
	
	LoopClosing* mpLoopCloser = nullptr;

    std::list<KeyFrame *> mlNewKeyFrames; ///< 等待处理的关键帧列表

    KeyFrame *mpCurrentKeyFrame =nullptr;

    std::list<MapPoint *> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;
    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
    
    // used in gravity refinement
    MatrixXd TangentBasis(Vector3d &g0);
    
    // Refine gravity direction after linear alignment
//     void RefineGravity(vector<KeyFrameInit*> &all_image_frame, Vector3d &g, VectorXd &x);
    
    // Linear Visual-Inertial alignemnt
//     bool LinearAlignment(vector<KeyFrameInit*> &all_image_frame, Vector3d &g, VectorXd &x);

    std::mutex mMutexGravity;	
  
    double mnStartTime;
    bool mbFirstTry;
    double mnVINSInitScale;
    Vector3d mGravityVec; // gravity vector in world frame

    std::mutex mMutexVINSInitFlag;
    bool mbVINSInited;

    std::mutex mMutexFirstVINSInitFlag;
    bool mbFirstVINSInited;

    unsigned int mnLocalWindowSize;
    unsigned int mnInitWindowSize;
    std::list<KeyFrame *> mlLocalKeyFrames;

    std::mutex mMutexMapUpdateFlag;
    bool mbMapUpdateFlagForTracking;
    KeyFrame *mpMapUpdateKF;

    bool mbUpdatingInitPoses;

    std::mutex mMutexCopyInitKFs;
    bool mbCopyInitKFs;

    bool GetFlagCopyInitKFs() {
	unique_lock<mutex> lock(mMutexCopyInitKFs);
	return mbCopyInitKFs;
    }

    void SetFlagCopyInitKFs(bool flag) {
	unique_lock<mutex> lock(mMutexCopyInitKFs);
	mbCopyInitKFs = flag;
    }
    
    ofstream output_gw, output_biasa, output_biasg, output_v;
    
    std::mutex mMutexMessageUpdate;
    bool bMessageUpdate;
    Vector3d _velocity;
    Vector3d _biasa, _biasg;
    Vector3d _gravity;
	
// 	list<vill_odom_nodelet::KeyFramePtr> keyframes_for_localization;
// 	list<vill_odom_nodelet::MapPointPtr> mappoints_for_localization;
// 	vill_odom_nodelet::LocalKeyframesPtr keyframes_for_localization;
// 	vill_odom_nodelet::LocalMappointsPtr mappoints_for_localization;
	
	int localizationState;
	std::mutex mMutexStateUpdate;
	
	std::mutex mMutexStateUpdateMappointsLocalization;
	bool bmappoints_updated_by_localization;
	DP* opt_mappoints;
	
	std::mutex mMutexStatedUpdateLocalKeyFramesLocalization;
	vector<Matrix4d> opt_localkeyframe_poses;
	vector<int> opt_ids;
	bool blocalkeyframes_updated_by_localization;
	
	
	static MatrixXd info;
	
	SE3f initialwi;
	
};

} //namespace vill

#endif // LOCALMAPPING_H
