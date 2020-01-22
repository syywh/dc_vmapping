#ifndef FRAME_H_
#define FRAME_H_


#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;

#include "thirdparty/DBoW2/DBoW2/BowVector.h"
#include "thirdparty/DBoW2/DBoW2/FeatureVector.h"

namespace vill {

    // the size of the grid. perhaps should be adjusted w.r.t. the size of images
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class MapPoint;
	
	class KeyFrame;

    class ORBextractor;

    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef enum {
            Monocular = 0, Stereo, RGBD
        } SensorType;

	
	//syywh
	int nmatches;
    
    public:

        Frame();

        // Copy constructor.
        Frame(const Frame &frame);

        // Constructor for stereo cameras.
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft,
              ORBextractor *extractorRight, ORBVocabulary *voc, Matrix3f &K, cv::Mat &distCoef, const float &bf,
              const float &thDepth);

        // Constructor for RGB-D cameras.
        Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor,
              ORBVocabulary *voc, Matrix3f &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular cameras.
        Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, 
			  Matrix3f &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

	
        // steps for extracting features
        void ExtractFeatures();

        // Extract ORB features on the image. 0 for left image and 1 for right image.
        // features in mvKeys, descriptors in mDescriptors
        void ExtractORB(int flag, const cv::Mat &im);

        // compute the image pyramid
        void ComputeImagePyramid();

        // Compute Bag of Words representation. === deleted? 
        // saved in mBowVec
        void ComputeBoW();

        // Set the camera pose.
        void SetPose(const SE3f &Tcw);
		
		void setLaserPose(const Matrix4f &Tlw);
		
		void setLaserposeTimestamp(const double &t);
		
		Matrix4f getLaserPose();
		
		double getLaserposeTimestamp();

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline Vector3f GetCameraCenter() {
            return mOw;
        }

        // Returns inverse of rotation
        inline Matrix3f GetRotationInverse() {
            return mRwc;
        }

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        // Check the pixels, distance and point of view
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        // Compute the cell of a keypoint (return false if outside the grid)
		// Check whether the keypoint is in the grid
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);


		// Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();
		
		// Get features in certain area.
		// should fill the grid with AssignFeaturesToGrid first
        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, 
										 const int minLevel = -1,const int maxLevel = -1) const;

        // Search a match for each keypoint in the left image to a keypoint in the right image.
        // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
		// match with ORB features first, then adjust
		// Rectify first, errors within two pixels
        void ComputeStereoMatches();

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
		// compute data in stereo style from RGBD data for uniform interfaces
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        Vector3f UnprojectStereo(const int &i);

        // Computes image bounds for the undistorted image (called in the constructor).
        // not used
        void ComputeImageBounds(const cv::Mat &imLeft);


        // coordinate transform: world, camera
        inline Vector3f World2Camera(const Vector3f &p_w, const SE3f &T_c_w) const {
            return T_c_w * p_w;
        }

        inline Vector3f Camera2World(const Vector3f &p_c, const SE3f &T_c_w) const {
            return T_c_w.inverse() * p_c;
        }

        // coordinate transform: camera, pixel
        inline Vector2f Camera2Pixel(const Vector3f &p_c) const {
            return Vector2f(
                    fx * p_c(0, 0) / p_c(2, 0) + cx,
                    fy * p_c(1, 0) / p_c(2, 0) + cy
            );
        }

        inline Vector3f Pixel2Camera(const Vector2f &p_p, double depth = 1) const {
            return Vector3f(
                    (p_p(0, 0) - cx) * depth / fx,
                    (p_p(1, 0) - cy) * depth / fy,
                    depth
            );
        }

		// coordinate transform: world, pixel
        inline Vector3f Pixel2World(const Vector2f &p_p, const SE3f &T_c_w, double depth = 1) const {
            return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
        }

        Vector2f World2Pixel(const Vector3f &p_w, const SE3f &T_c_w) const {
            return Camera2Pixel(World2Camera(p_w, T_c_w));
        }

        // === IMU related
        // compute the imu preintegration
//         void ComputeIMUPreIntSinceLastFrame(const Frame *pLastF, IMUPreintegrator &imupreint) const;

        // update the pose matrix from navigation state
        void UpdatePoseFromNS(const SE3d &Tbc);

        // update the nav state from camera pose
        void UpdateNSFromPose();

        // set initial navigation, and set bias to zero
//         void SetInitialNavStateAndBias(const NavState &ns);

        // update the navigation status using preintegration
//         void UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw);

//         // get navigation state
//         NavState GetNavState(void) const {
//             return mNavState;
//         }
// 
//         // set navigation state
//         void SetNavState(const NavState &ns) {
//             mNavState = ns;
//         }

        // set gyro bias
//         void SetNavStateBiasGyr(const Vector3d &bg);

        // set accelerate bias
//         void SetNavStateBiasAcc(const Vector3d &ba);


    public:
        // Vocabulary used for relocalization.
        ORBVocabulary *mpORBvocabulary = nullptr;     

        // Feature extractor. The right is used only in the stereo case.
        ORBextractor *mpORBextractorLeft = nullptr;
        ORBextractor *mpORBextractorRight = nullptr;     

        cv::Mat mImGray;    // left
        cv::Mat mImRight;   // right
        cv::Mat mImDepth;   // rgbd depth

        // image pyramid
        vector<cv::Mat> mvImagePyramid;

        // Frame timestamp.
        double mTimeStamp = -1;

        // Calibration matrix and OpenCV distortion parameters.
        Matrix3f mK = Matrix3f::Zero();
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;

        static Mat map1, map2;  // for undistortion
        cv::Mat mDistCoef;  
        static bool mbNeedUndistort;        // whether the input images are undistorted or not

        // Stereo baseline multiplied by fx.
        float mbf = 0;       

        // Stereo baseline in meters.
        float mb;                   

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;             

        // Number of KeyPoints.
        int N; ///< KeyPoints数量

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        // mvKeys: features in the left image
        // mvKeysRight: features in the right image
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
		
		// keyframes from which the features are extracted
        vector<int> mvMatchedFrom;  

        // whether features are extracted
        bool mbFeatureExtracted = false;    // flag to indicate if the ORB features are detected

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        // u coordinate for the corresponding feature in the right image
        // mvDepth -- the depth information
        // -1 for monocular
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.
        std::vector<MapPoint *> mvpMapPoints;

        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        // the inverse of the grids' coordinates
		// for quick searching
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

        // divide the image into grids
        // #define FRAME_GRID_ROWS 48
        // #define FRAME_GRID_COLS 64
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
        bool mbGridSet = false;       // the grids are set or not

        // Camera pose.
        SE3f mTcw;  // World 到 Camera
        bool mbPoseSet = false;      // Pose set or not

        // Current and Next Frame id.
        static long unsigned int nNextId; ///< Next Frame id.
        long unsigned int mnId = 0; ///< Current Frame id.

        // Reference Keyframe pointer.
        KeyFrame *mpReferenceKF = nullptr;

        // Scale pyramid info.
        int mnScaleLevels = 0;//图像提金字塔的层数
        float mfScaleFactor = 0;//图像提金字塔的尺度因子
        float mfLogScaleFactor; // 对数scale缩放值

        // parameters for each level of pyramid
        vector<float> mvScaleFactors;               // 缩放倍数
        vector<float> mvInvScaleFactors;            // 倒数
        vector<float> mvLevelSigma2;                // 平方
        vector<float> mvInvLevelSigma2;             // 倒数平方

        // Undistorted Image Bounds (computed once).
        // for grids division
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

        // IMU Data from last Frame to this Frame
//         std::vector<IMUData> mvIMUDataSinceLastFrame;

        // For pose optimization, use as prior and prior information(inverse covariance)
        // priori determined by IMU
//         Matrix<double, 15, 15> mMargCovInv = Matrix<double, 15, 15>::Zero();
//         NavState mNavStatePrior;

        // sensors
        SensorType mSensor;

        // Rotation, translation and camera center
        // derived from camera pose
        Matrix3f mRcw; ///< Rotation from world to camera
        Vector3f mtcw; ///< Translation from world to camera
        Matrix3f mRwc; ///< Rotation from camera to world
        Vector3f mOw; //==mtwc,Translation from camera to world
        
        //added correlated pose
        Matrix4f mTlaserpose;
		double mLaserposeTimestamp;
        

        // states from IMU，Rwb, twb, v, ba, bg,共15维，存储在NavState当中
//         NavState mNavState; // Navigation state used in VIO
    };

}// namespace vill

#endif // FRAME_H
