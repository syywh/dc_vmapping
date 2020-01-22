#ifndef YGZ_MAPPOINT_H
#define YGZ_MAPPOINT_H

#include <Eigen/Core>
using namespace Eigen;

#include <opencv2/core/core.hpp>
using namespace cv;

#include <mutex>
#include <map>


// 地图点

namespace vill {

    class KeyFrame;

    class Map;

    class Frame;

// 现在这个 map point 锁的有点太频繁
    class MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MapPoint(const Vector3f &Pos, KeyFrame *pRefKF, Map *pMap);

        MapPoint(const Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF);
		
		MapPoint(Map* pMap);

        std::map<KeyFrame *, size_t> GetObservations();

        int Observations();

        void AddObservation(KeyFrame *pKF, size_t idx);

        void EraseObservation(KeyFrame *pKF);

        int GetIndexInKeyFrame(KeyFrame *pKF);

        bool IsInKeyFrame(KeyFrame *pKF);

        void SetBadFlag();

        bool isBad();

        void Replace(MapPoint *pMP);

        MapPoint *GetReplaced();

        void IncreaseVisible(int n = 1);

        void IncreaseFound(int n = 1);

        float GetFoundRatio();

        inline int GetFound() {
            return mnFound;
        }

        void ComputeDistinctiveDescriptors();

        cv::Mat GetDescriptor();

        void UpdateNormalAndDepth();

        float GetMinDistanceInvariance();

        float GetMaxDistanceInvariance();

        int PredictScale(const float &currentDist, KeyFrame *pKF);

        int PredictScale(const float &currentDist, Frame *pF);

        void UpdateScale(float scale) {
            SetWorldPos(mWorldPos * scale);
            mfMinDistance *= scale;
            mfMaxDistance *= scale;
        }

        // accessors, will be locked
        void SetWorldPos(const Vector3f &Pos);

        Vector3f GetWorldPos();

        Vector3f GetNormal();

        KeyFrame *GetReferenceKeyFrame();

		float getmfMinDistance(){return mfMinDistance;}
		void setmfMinDistance(float dist){mfMinDistance = dist;}
		float getmfMaxDistance(){return mfMaxDistance;}
		void setmfMaxDistance(float dist){mfMaxDistance = dist;}
		
		void setNormal(Eigen::Vector3f& normal){mNormalVector = normal;}
		void setDescriptor(cv::Mat& desc){desc.copyTo(mDescriptor);}
		void setRefKF(KeyFrame* pKF){mpRefKF = pKF;};
		
    public:
        long unsigned int mnId = 0; ///< Global ID for MapPoint
        static long unsigned int nNextId;
        long int mnFirstKFid = -1; ///< 创建该MapPoint的关键帧ID
        long int mnFirstFrame = -1; ///< 创建该MapPoint的帧ID（即每一关键帧有一个帧ID）
        int nObs = 0;

        // Variables used by the tracking
        // predicted position in tracking
        float mTrackProjX = 0;
        float mTrackProjY = 0;
        float mTrackProjXR = 0;
        bool mbTrackInView = false;
        int mnTrackScaleLevel = 0;
        float mTrackViewCos = 0;
        long unsigned int mnTrackReferenceForFrame = 0;
        long unsigned int mnLastFrameSeen = 0;

        // Variables used by local mapping
        long unsigned int mnBALocalForKF = 0;
        long unsigned int mnFuseCandidateForKF = 0;

        // Variables used by loop closing
        long unsigned int mnLoopPointForKF = 0;
        long unsigned int mnCorrectedByKF = 0;
        long unsigned int mnCorrectedReference = 0;
        long unsigned int mnBAGlobalForKF = 0;
        Vector3f mPosGBA = Vector3f(0, 0, 0);

        static std::mutex mGlobalMutex; // 整个地图的锁
        
        bool covariace_recovered;
		double covariance_z;
			
		std::map<int ,size_t> mReObservations;
		
		std::map<int, int> re_mObs;
		
		long unsigned int refKFid;
		
		
		

    private:

        // Position in absolute coordinates
        Vector3f mWorldPos; ///< MapPoint在世界坐标系下的坐标

        // Keyframes observing the point and associated index in keyframe
        std::map<KeyFrame *, size_t> mObservations;

        // Mean viewing direction
        Vector3f mNormalVector = Vector3f(0, 0, 0);    // 法线 or 观测角

        // Best descriptor to fast matching
        cv::Mat mDescriptor; ///< 通过 ComputeDistinctiveDescriptors() 得到的最优描述子

        // Reference KeyFrame
        KeyFrame *mpRefKF = nullptr;     // 如果reference被cull掉，是否应该这个地图点也删掉？
        
        float mfInvDepth = 0;           // Reference frame 中的逆深度
        float mfInvDepthMax = 0, mfInvDepthMin = 0; // 逆深度的最大最小值

        // Tracking counters
        int mnVisible = 1;
        int mnFound = 1;

        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad = false;
        MapPoint *mpReplaced = nullptr;

        // Scale invariance distances
        float mfMinDistance = 0;
        float mfMaxDistance = 0;

        Map *mpMap = nullptr;

        std::mutex mMutexPos;      // 位置锁
        std::mutex mMutexFeatures; // Feature的锁

    };

} //namespace ORB_SLAM

#endif // MAPPOINT_H
