#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include "ORBVocabulary.h"
#include "ORBextractor.h"

// #include "IMU/imudata.h"
// #include "IMU/NavState.h"
// #include "IMU/IMUPreintegrator.h"
// #include "MapLine.h"
// #include "Linematcher.h"
// #include <LineStructure.hh>

#include <set>
#include <mutex>

namespace vill {

    class Map;

    class MapPoint;

    class Frame;
	
// 	class MapLine;

    class KeyFrameDatabase;

    class KeyFrame {
    public:
	void setInitKF(bool flag);
	bool isInitKF();
	void EraseImage();
	
	// syywh
	int nmatches;

    private:
	bool bIsInitKF;
	std::mutex mMutexInitKF;
	std::mutex mMutexEraseImage;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // constructed from frame
        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

        // VIO constructor
//         KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, std::vector<IMUData> vIMUData, KeyFrame *pLastKF = NULL);
		
		//localization constructor
		KeyFrame(Map* pMap, Frame& F);

        // Pose functions
        // mutex
        void SetPose(const SE3f &Tcw);
		
		void SetLaserPose(const Matrix4f &Tlw);
		
		void SetLaserposeTimestamp(const double& timestamplaser);
		
		double GetLaserposeTimestamp();

        SE3f GetPose();
		
		Matrix4f GetLaserpose();
	
		SE3f GetPoseOri();

        SE3f GetPoseInverse();
	
		SE3f GetPoseInverseOri();

        Vector3f GetCameraCenter();

        Vector3f GetStereoCenter();

        Matrix3f GetRotation();

        Vector3f GetTranslation();

        // Bag of Words Representation ===
        void ComputeBoW();

        // Covisibility graph functions
        void AddConnection(KeyFrame *pKF, const int &weight);

        // cov graph operations
        void EraseConnection(KeyFrame *pKF);

        void UpdateConnections();

        void UpdateBestCovisibles();

        std::set<KeyFrame *> GetConnectedKeyFrames();

        std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();

        std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);

        std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);

        int GetWeight(KeyFrame *pKF);

        // Spanning tree functions
        void AddChild(KeyFrame *pKF);

        void EraseChild(KeyFrame *pKF);

        void ChangeParent(KeyFrame *pKF);

        std::set<KeyFrame *> GetChilds();

        KeyFrame *GetParent();

        bool hasChild(KeyFrame *pKF);

        // Loop Edges
        void AddLoopEdge(KeyFrame *pKF);

        std::set<KeyFrame *> GetLoopEdges();

        // MapPoint observation functions
        void AddMapPoint(MapPoint *pMP, const size_t &idx);
//         void AddMapLine(MapLine *pML, unsigned int segid);
// 		void AddMapLineDesc(LinesVec &lvec, unsigned int segid);

        void EraseMapPointMatch(const size_t &idx);

        void EraseMapPointMatch(MapPoint *pMP);

        void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);

        std::set<MapPoint *> GetMapPoints();

        std::vector<MapPoint *> GetMapPointMatches();
		
// 		std::vector<MapLine *> GetMapLineMatches();
// 		ScaleLines GetMapLineDesc(){return mvpMapLineDescs;}
		

        int TrackedMapPoints(const int &minObs);

        MapPoint *GetMapPoint(const size_t &idx);

        // KeyPoint functions
        std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;

		// Compute the 3D point in world coordinate if the depth exists
        Vector3f UnprojectStereo(int i);

        // Image
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();

        void SetErase();

        // Set/check bad flag
        void SetBadFlag();

        bool isBad();

        // Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        static bool weightComp(int a, int b) {
            return a > b;
        }

        static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
            return pKF1->mnId < pKF2->mnId;
        }

        // coordinate transform: world, camera, pixel
        inline Vector3f World2Camera(const Vector3f &p_w, const SE3f &T_c_w) const {
            return T_c_w * p_w;
        }

        inline Vector3f Camera2World(const Vector3f &p_c, const SE3f &T_c_w) const {
            return T_c_w.inverse() * p_c;
        }

        inline Vector2f Camera2Pixel(const Vector3f &p_c) const {
            return Vector2f(
                    fx * p_c(0, 0) / p_c(2, 0) + cx,
                    fy * p_c(1, 0) / p_c(2, 0) + cy
            );
        }

        inline Vector3f Pixel2Camera(const Vector2f &p_p, float depth = 1) const {
            return Vector3f(
                    (p_p(0, 0) - cx) * depth / fx,
                    (p_p(1, 0) - cy) * depth / fy,
                    depth
            );
        }

        inline Vector3f Pixel2World(const Vector2f &p_p, const SE3f &T_c_w, float depth = 1) const {
            return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
        }

        inline Vector2f World2Pixel(const Vector3f &p_w, const SE3f &T_c_w) const {
            return Camera2Pixel(World2Camera(p_w, T_c_w));
        }


        KeyFrame *GetPrevKeyFrame(void);

        KeyFrame *GetNextKeyFrame(void);

        void SetPrevKeyFrame(KeyFrame *pKF);

        void SetNextKeyFrame(KeyFrame *pKF);

//         std::vector<IMUData> GetVectorIMUData(void);

//         void AppendIMUDataToFront(KeyFrame *pPrevKF);

//         void ComputePreInt(void);

//         const IMUPreintegrator &GetIMUPreInt(void);

        void UpdateNavStatePVRFromTcw(const SE3d &Tcw, const SE3d &Tbc);

        void UpdatePoseFromNS(const SE3d &Tbc);

//         void UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw);

//         void SetNavState(const NavState &ns);

//         const NavState &GetNavState(void);

//         void SetNavStateVel(const Vector3d &vel);
// 
//         void SetNavStatePos(const Vector3d &pos);
// 
//         void SetNavStateRot(const Matrix3d &rot);
// 
//         void SetNavStateRot(const SO3d &rot);
// 
//         void SetNavStateBiasGyr(const Vector3d &bg);
// 
//         void SetNavStateBiasAcc(const Vector3d &ba);
// 
//         void SetNavStateDeltaBg(const Vector3d &dbg);
// 
//         void SetNavStateDeltaBa(const Vector3d &dba);
// 
//         void SetInitialNavStateAndBias(const NavState &ns);
		
		void SetInitializedByVIO();
		void ResetInitializedByVIO();
		bool GetInitializedByVIO();
		
		
		map<int, int> getConnectedKeyFrameWeights();

		void addmvpOrderedConnectedKFs(KeyFrame* pKF){mvpOrderedConnectedKeyFrames.push_back(pKF);}
		
		void addmConnectedKF_weights(KeyFrame* pKF, int weight ){mConnectedKeyFrameWeights[pKF] = weight;}
		
		void addMapPoint(MapPoint* pMP){mvpMapPoints.push_back(pMP);}

        // Variables used by loop closing
//         NavState mNavStateGBA;       //mTcwGBA
//         NavState mNavStateBefGBA;    //mTcwBefGBA
        
        

    public:
      
      Matrix4d Tlc;

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
        static long unsigned int nNextId;

        long unsigned int mnId =0;


        // constructed from this frame
        const long unsigned int mnFrameId;

        // timestamp
        const double mTimeStamp =0;

        // Grid (to speed up feature matching)
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;
        const float mfGridElementHeightInv;

        // Variables used by the tracking
        long unsigned int mnTrackReferenceForFrame = 0;
        long unsigned int mnFuseTargetForKF = 0;

        // Variables used by the local mapping
        long unsigned int mnBALocalForKF = 0;
        long unsigned int mnBAFixedForKF = 0;

        // Variables used by the keyframe database
        long unsigned int mnLoopQuery = 0;
        int mnLoopWords = 0;
        float mLoopScore = 0;
        long unsigned int mnRelocQuery = 0;
        int mnRelocWords = 0;
        float mRelocScore = 0;

        // Variables used by loop closing
        SE3f mTcwGBA;
        SE3f mTcwBefGBA;
        long unsigned int mnBAGlobalForKF;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
		int lineimg_width,lineimg_height; // img size when extract descriptors
		std::vector<unsigned int>Temp2SegID;
		std::map<unsigned int, unsigned int> segID2MapLine; // segID MapLineID

        // Number of KeyPoints
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const std::vector<cv::KeyPoint> mvKeys;
        const std::vector<float> mvuRight; // negative value for monocular points
        const std::vector<float> mvDepth; // negative value for monocular points
        const cv::Mat mDescriptors;

        //BoW
        DBoW2::BowVector mBowVec; ///< Vector of words to represent images
        DBoW2::FeatureVector mFeatVec; ///< Vector of nodes with indexes of local features

        // Pose relative to parent (this is computed when bad flag is activated)
        SE3f mTcp;

        // Scale
        const int mnScaleLevels;
        const float mfScaleFactor;
        const float mfLogScaleFactor;
        const std::vector<float> mvScaleFactors;
        const std::vector<float> mvLevelSigma2;
        const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;
        const Matrix3f mK;

        // image pyramid
        vector<cv::Mat> mvImagePyramid;
		void AddKeyFrameImage(cv::Mat img){mvImagePyramid.push_back(img);}

	
		//syywh perhaps not used
		// Check whether the keyframe is updated by the laser map
		bool isUpdated();
		void resetUpdated();
		void setUpdated();
		bool updated;
		
// 		void getIMUData(vector<IMUData>& vIMUdata_);
		
		// Grid over the image to speed up feature matching
		//FIXME changed from protected
		std::vector< std::vector <std::vector<size_t> > > mGrid;
		// The following variables need to be accessed trough a mutex to be thread safe.
		
		vector<int> re_mvpOrderedConnectedKFs;
		map<int, int> mConnectedKF_weights;
		vector<int> re_vMapPoints;
		long unsigned int pre_id;
		
		//used for depth completion loss
		std::map<pair<int,int>, int> m_pt_mpid;//reproject map  points to this keyframe
		cv::Mat gradient_for_mp;
		float loss;
		
    protected:
      
      //for saving
      
	SE3f Tcw_ori;
	SE3f Twc_ori;
	Vector3f Ow_ori;
      
        // The following variables need to be accessed trough a mutex to be thread safe.
        // data used in imu
        std::mutex mMutexPrevKF;
        std::mutex mMutexNextKF;
        KeyFrame *mpPrevKeyFrame;
        KeyFrame *mpNextKeyFrame;

        // P, V, R, bg, ba, delta_bg, delta_ba (delta_bx is for optimization update)
//         std::mutex mMutexNavState;
//         NavState mNavState;

        // IMU Data from lask KeyFrame to this KeyFrame
//         std::mutex mMutexIMUData;
//         std::vector<IMUData> mvIMUData;
//         IMUPreintegrator mIMUPreInt;

		
		//initialized by VIO init
		std::mutex mMutexInitVIO;
		bool initializedByVIO;
		
		
        // SE3 Pose and camera center
        SE3f Tcw;
        SE3f Twc;
        Vector3f Ow;
        Vector3f Cw; // Stereo middel point. Only for visualization

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpMapPoints;
// 		std::vector<MapLine *> mvpMapLines;
// 		ScaleLines mvpMapLineDescs;

        // BoW
        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBvocabulary;

//         // Grid over the image to speed up feature matching
//         std::vector<std::vector<std::vector<size_t> > > mGrid;

        std::map<KeyFrame *, int> mConnectedKeyFrameWeights; ///< 与该关键帧连接的关键帧与权重
        std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames; ///< 排序后的关键帧
        std::vector<int> mvOrderedWeights; ///< 排序后的权重(从大到小)

        // Spanning Tree and Loop Edges
        // std::set是集合，相比vector，进行插入数据这样的操作时会自动排序
        bool mbFirstConnection = false;
        KeyFrame *mpParent = nullptr;
        std::set<KeyFrame *> mspChildrens;
        std::set<KeyFrame *> mspLoopEdges;

        // Bad flags
        bool mbNotErase = false;
        bool mbToBeErased = false;
        bool mbBad = false;

        float mHalfBaseline =0; // Only for visualization

        Map *mpMap =nullptr;

        std::mutex mMutexPose;
        std::mutex mMutexConnections;
        std::mutex mMutexFeatures;
	
		int save_updated;
		
		Matrix4f mlaserPose;
		double laserPoseTimestamp;

    };

} //namespace vill

#endif // KEYFRAME_H
