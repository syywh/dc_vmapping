#ifndef MAPDRAWER_H_
#define MAPDRAWER_H_

#include "Map.h"
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using namespace Sophus;

#include <pangolin/pangolin.h>
#include <string>
using namespace std;


namespace vill {

    class MapDrawer {
    public:
	void DrawCurrentIMUFrame(pangolin::OpenGlMatrix &Twb);
	void DrawExtrinsicCalibration(pangolin::OpenGlMatrix &Twc, pangolin::OpenGlMatrix &Twb);
	void SetCurrentIMUMatrix(const SE3d &Twb);
	void GetCurrentOpenGLIMUMatrix(pangolin::OpenGlMatrix &M);
      
    private:
	SE3d mIMUMatrix;
	
    public:
        MapDrawer(Map *pMap/*, LaserMap* pLaserMap*/, const string &strSettingPath);
// 		MapDrawer(Map *pMap, LaserMap* pLaserMap, const string &strSettingPath);

        Map *mpMap;
// 		LaserMap* mpLaserMap;

        void DrawMapPoints();
	
		void DrawLaserMap();
	
// 		void DrawMatches();

        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
// 		void DrawKeyFrames_inLaserMap(const bool bDrawKF, const bool bDrawGraph);

        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
		
		void DrawCurrentLaserPose(pangolin::OpenGlMatrix &Twl);

        void SetCurrentCameraPose(const SE3d &Tcw);
		
		void SetCurrentLaserPose(const Matrix4f &Tlw);

        void SetReferenceKeyFrame(KeyFrame *pKF);

        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
		
		void GetCurrentOpenGLLaserMatrix(pangolin::OpenGlMatrix &M);

    private:

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
		float mMatchedPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        SE3d mCameraPose;
		
		Matrix4f mLaserPose;
		float mLaserMapPointSize;

        std::mutex mMutexCamera;
		std::mutex mMutexLaser;
    };

} //namespace vill

#endif // MAPDRAWER_H
