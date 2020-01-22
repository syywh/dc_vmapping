#include "Viewer.h"
#include "System.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include <pangolin/pangolin.h>

namespace vill {
    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
                   const string &strSettingPath) :
            mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1) {
            mImageWidth = 640;
            mImageHeight = 480;
        }

		// TODO: Ҫ��FrameDrawer��ͼ�񱣳�һ�£����� 

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }
    
    Viewer::~Viewer(){
		pangolin::DestroyWindow("Vill: Map Viewer");
	}

    void Viewer::Run() {
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("Vill: Map Viewer", 1024, 768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
		pangolin::Var<bool> menuShowLaserPoints("menu.Show LaserPoints",false,false);
		pangolin::Var<bool> menuShowMatches("menu.Show Matches",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
		pangolin::Var<bool> menuShowLaserpose("menu.Show laser pose",false,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc, Twb, Twl;
        Twc.SetIdentity();
		Twb.SetIdentity();
		Twl.SetIdentity();

        cv::namedWindow("VILL: Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = false;
		bool vwopened = false;
        while (1) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
	    mpMapDrawer->GetCurrentOpenGLIMUMatrix(Twb);
// 		mpMapDrawer->GetCurrentOpenGLLaserMatrix(Twl);

            if (menuFollowCamera && bFollow) {
                s_cam.Follow(Twc);
            } else if (menuFollowCamera && !bFollow) {
                s_cam.SetModelViewMatrix(
                        pangolin::ModelViewLookAt(mViewpointX, -mViewpointY, -mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            } else if (!menuFollowCamera && bFollow) {
                bFollow = false;
            }
            
//             if(bUpdateVP)
// 	    {
// 		if(menuFollowCamera)
// 		{
// 		  s_cam.SetModelViewMatrix(
// 			  pangolin::ModelViewLookAt(mViewpointX, -mViewpointY, -mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
// 		  s_cam.Follow(Twc);
// 		}
// 		bUpdateVP = false;
// 	    }

            if (menuLocalizationMode && !bLocalizationMode) {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            } else if (!menuLocalizationMode && bLocalizationMode) {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

		d_cam.Activate(s_cam);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		mpMapDrawer->DrawCurrentCamera(Twc);
// 	    mpMapDrawer->DrawCurrentIMUFrame(Twb);
// 	    mpMapDrawer->DrawExtrinsicCalibration(Twc, Twb);
		if (menuShowKeyFrames || menuShowGraph)
// 			if(!menuShowLaserPoints)
			mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
// 		else
// 			mpMapDrawer->DrawKeyFrames_inLaserMap(menuShowKeyFrames, menuShowGraph);
		if (menuShowPoints)
			mpMapDrawer->DrawMapPoints();
// 	    if(menuShowLaserPoints){
// 		    mpMapDrawer->DrawLaserMap();
// 	    }
// 	    if(menuShowMatches){
// 	      mpMapDrawer->DrawMatches();
// 	    }
// 	    if(menuShowLaserpose){
// 			mpMapDrawer->DrawCurrentLaserPose(Twl);
// 		}
	    
		pangolin::FinishFrame();

		cv::Mat im = mpFrameDrawer->DrawFrame();
	    cv::Mat im_resize;
// 	    cv::resize(im, im_resize,Size(640, 480));
            cv::imshow("VILL: Current Frame", im); 
            cv::waitKey(mT);

            if (menuReset) {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
// 				menuShowLaserPoints = true;
				menuShowLaserpose = false;
                menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if (Stop()) {
                while (isStopped()) {
					std::this_thread::sleep_for(std::chrono::milliseconds(3));

                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::setViewPoint(const float& vx, const float& vy, const float& vz)
    {
	unique_lock<mutex> lock(mMutexVP);
	mViewpointX = vx;
	mViewpointY = vy;
	mViewpointZ = vz;
	if(!bUpdateVP)
	  bUpdateVP = true;
    }
    
    void Viewer::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop() {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested) {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

} // namespace vill
