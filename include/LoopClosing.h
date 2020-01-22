#ifndef LOOPCLOSING_H_
#define LOOPCLOSING_H_

#include "ORBVocabulary.h"

// #include "Thirdparty/g2o/g2o/types/sim3.h"
#include "g2o/types/sim3/sim3.h"

#include <set>
#include <list>
#include <mutex>
#include <thread>
using namespace std;

// Loop closing 线程
// 基本没动，除了加IMU部分

namespace vill {

    class Tracking;

    class LocalMapping;

    class KeyFrameDatabase;

    class KeyFrame;

    class Map;

    class MapPoint;

    class LoopClosing {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef pair<set<KeyFrame *>, int> ConsistentGroup;
        typedef map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
                Eigen::aligned_allocator<std::pair<const KeyFrame *, g2o::Sim3> > > KeyFrameAndPose;

        LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, ConfigParam *pParams);

        void SetTracker(Tracking *pTracker);

        void SetLocalMapper(LocalMapping *pLocalMapper);

        // Main function
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        void RequestReset();

        // This function will run in a separate thread
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        bool isRunningGBA() {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbRunningGBA;
        }

        bool isFinishedGBA() {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbFinishedGBA;
        }

        void RequestFinish();

        bool isFinished();

        ConfigParam *mpParams;
        bool mbUseIMU;

        bool GetMapUpdateFlagForTracking();

        void SetMapUpdateFlagInTracking(bool bflag);
		
		map<pair<long unsigned int, long unsigned int>, cv::Mat> mLoops;

    protected:

        bool CheckNewKeyFrames();

        bool DetectLoop();

        bool ComputeSim3();

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

        void CorrectLoop();

        void ResetIfRequested();

        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap;
        Tracking *mpTracker;

        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBVocabulary;

        LocalMapping *mpLocalMapper;

        std::list<KeyFrame *> mlpLoopKeyFrameQueue;

        std::mutex mMutexLoopQueue;

        // Loop detector parameters
        float mnCovisibilityConsistencyTh;

        // Loop detector variables
        KeyFrame *mpCurrentKF;
        KeyFrame *mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;
        std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
        std::vector<KeyFrame *> mvpCurrentConnectedKFs;
        std::vector<MapPoint *> mvpCurrentMatchedPoints;
        std::vector<MapPoint *> mvpLoopMapPoints;
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;

        long unsigned int mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        bool mbRunningGBA = false;
        bool mbFinishedGBA = true;
        bool mbStopGBA = false;
        std::mutex mMutexGBA;
        std::thread *mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        bool mbFixScale;
        bool mnFullBAIdx;

        std::mutex mMutexMapUpdateFlag;
        bool mbMapUpdateFlagForTracking = false;
    };

} //namespace vill

#endif // LOOPCLOSING_H
