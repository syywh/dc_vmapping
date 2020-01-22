#ifndef VIEWER_H_
#define VIEWER_H_

#include <mutex>
#include <thread>
#include <string>
#include "opencv2/highgui/highgui.hpp"

using namespace std;

// 可视化类，基本没动
namespace vill {

    class Tracking;

    class FrameDrawer;

    class MapDrawer;

    class System;

    class Viewer {
    public:
        Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
               const string &strSettingPath);
		
		~Viewer();

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();
	
	void setViewPoint(const float &vx, const float &vy, const float &vz);

    private:

	bool bUpdateVP;
      
        bool Stop();

        System *mpSystem;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        Tracking *mpTracker;

        // 1/fps in ms

        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;
	
	std::mutex mMutexVP;

    };

}


#endif // VIEWER_H
	

