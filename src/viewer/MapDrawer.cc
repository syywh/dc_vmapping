#include "MapDrawer.h"
#include "MapPoint.h"

#include "KeyFrame.h"
#include "glog/logging.h"

namespace vill {

    MapDrawer::MapDrawer(Map *pMap/*, LaserMap* pLaserMap*/, const string &strSettingPath) : mpMap(pMap) /*, mpLaserMap(pLaserMap)*/{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
		mMatchedPointSize = fSettings["Viewer.MatchedPointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
// 		mLaserMapPointSize = fSettings["Viewer.LaserMapPointSize"];

    }
    

//     MapDrawer::MapDrawer(Map *pMap, LaserMap* pLaserMap, const string &strSettingPath) : mpMap(pMap) , mpLaserMap(pLaserMap){
//         cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
// 
//         mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
//         mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
//         mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
//         mPointSize = fSettings["Viewer.PointSize"];
// 		mMatchedPointSize = fSettings["Viewer.MatchedPointSize"];
//         mCameraSize = fSettings["Viewer.CameraSize"];
//         mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
// 		mLaserMapPointSize = fSettings["Viewer.LaserMapPointSize"];
// 
//     }
    
    void MapDrawer::SetCurrentIMUMatrix(const SE3d& Twb)
    {
	unique_lock<mutex> lock(mMutexCamera);
	mIMUMatrix = Twb;
    }
    
    void MapDrawer::GetCurrentOpenGLIMUMatrix(pangolin::OpenGlMatrix& M)
    {
	if (mIMUMatrix.log().norm() > 1e-12) {
            Matrix3d Rwb;
            Vector3d twb;
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwb = mIMUMatrix.rotationMatrix();
                twb = mIMUMatrix.translation();
            }

            M.m[0] = Rwb(0, 0);
            M.m[1] = Rwb(1, 0);
            M.m[2] = Rwb(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwb(0, 1);
            M.m[5] = Rwb(1, 1);
            M.m[6] = Rwb(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwb(0, 2);
            M.m[9] = Rwb(1, 2);
            M.m[10] = Rwb(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twb(0);
            M.m[13] = twb(1);
            M.m[14] = twb(2);
            M.m[15] = 1.0;
        } else
            M.SetIdentity();
    }
    
    void MapDrawer::DrawCurrentIMUFrame(pangolin::OpenGlMatrix& Twb)
    {
// 	const float &w = mCameraSize;
//         const float h = w * 0.75;
//         const float z = w * 0.6;

        glPushMatrix();
	
	pangolin::OpenGlMatrix Twb_ = /*mpLaserMap->Tlc **/ Twb;

#ifdef HAVE_GLES
        glMultMatrixf(Twb_.m);
#else
        glMultMatrixd(Twb_.m);
#endif
	

        glLineWidth(5);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
        
	//x
	glVertex3f(0, 0, 0);
        glVertex3f(0.1, 0, 0);
	
	glEnd();
	
	glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
	
	//y
	glVertex3f(0, 0, 0);
        glVertex3f(0, 0.1, 0);
	
	glEnd();
	
	glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
	
	//z
	glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 0.1);
	
        glEnd();

        glPopMatrix();
    }

    void MapDrawer::DrawExtrinsicCalibration(pangolin::OpenGlMatrix &Twc, pangolin::OpenGlMatrix &Twb)
    {
	glLineWidth(5);
	glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	
	pangolin::OpenGlMatrix Twc_ = /*mpLaserMap->Tlc * */Twc;
	pangolin::OpenGlMatrix Twb_ =/* mpLaserMap->Tlc * */Twb;
	
	glVertex3f(Twc_.m[12], Twc_.m[13], Twc_.m[14]);
	glVertex3f(Twb_.m[12], Twb_.m[13], Twb_.m[14]);
	
	glEnd();
    }

    void MapDrawer::DrawMapPoints() {
        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
	
// 	Matrix3f rotlc = mpLaserMap->Tlc.block<3,3>(0,0).cast<float>();
// 	Vector3f translc = mpLaserMap->Tlc.block<3,1>(0,3).cast<float>();

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            Vector3f pos = vpMPs[i]->GetWorldPos();
// 	    pos = rotlc * pos + translc;
            glVertex3f(pos[0], pos[1], pos[2]);
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            Vector3f pos = (*sit)->GetWorldPos();
// 	    pos = rotlc * pos + translc;
            glVertex3f(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }
    

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
	
// 	Matrix4f Tlcf;
// 	Tlcf =(( mpLaserMap->Tlc)).cast<float>();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Matrix4f Twc = pKF->GetPoseInverse().matrix();
// 				Twc = pKF->Tlc.cast<float>() * Twc;

                glPushMatrix();

                glMultMatrixf(Twc.data());

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }

        if (bDrawGraph) {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            for (size_t i = 0; i < vpKFs.size(); i++) {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                Vector3f Ow = vpKFs[i]->GetCameraCenter();
// 		Ow = vpKFs[i]->Tlc.cast<float>().block<3,3>(0,0) * Ow + vpKFs[i]->Tlc.cast<float>().block<3,1>(0,3);
                if (!vCovKFs.empty()) {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        Vector3f Ow2 = (*vit)->GetCameraCenter();
// 			Ow2 = (*vit)->Tlc.cast<float>().block<3,3>(0,0) * Ow2+ (*vit)->Tlc.cast<float>().block<3,1>(0,3);
                        glVertex3f(Ow[0], Ow[1], Ow[2]);
                        glVertex3f(Ow2[0], Ow2[1], Ow2[2]);
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
                    Vector3f Owp = pParent->GetCameraCenter();
// 		    Owp = pParent->Tlc.cast<float>().block<3,3>(0,0) * Owp+ pParent->Tlc.cast<float>().block<3,1>(0,3);
                    glVertex3f(Ow[0], Ow[1], Ow[2]);
                    glVertex3f(Owp[0], Owp[1], Owp[2]);
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Vector3f Owl = (*sit)->GetCameraCenter();
// 		    Owl = (*sit)->Tlc.cast<float>().block<3,3>(0,0) * Owl+ (*sit)->Tlc.cast<float>().block<3,1>(0,3);
                    glVertex3f(Ow[0], Ow[1], Ow[2]);
                    glVertex3f(Owl[0], Owl[1], Owl[2]);
                }
            }

            glEnd();
        }
    }
    
//     void MapDrawer::DrawKeyFrames_inLaserMap(const bool bDrawKF, const bool bDrawGraph) {
//         const float &w = mKeyFrameSize;
//         const float h = w * 0.75;
//         const float z = w * 0.6;
// 
//         const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
// 	
// 	Matrix4f Tlcf;
// 	Tlcf =(( mpLaserMap->Tlc)).cast<float>();
// 
//         if (bDrawKF) {
//             for (size_t i = 0; i < vpKFs.size(); i++) {
//                 KeyFrame *pKF = vpKFs[i];
//                 Matrix4f Twc = pKF->GetPoseInverse().matrix();
// 				Twc = pKF->Tlc.cast<float>() * Twc;
// 
//                 glPushMatrix();
// 
//                 glMultMatrixf(Twc.data());
// 
//                 glLineWidth(mKeyFrameLineWidth);
//                 glColor3f(0.0f, 0.0f, 1.0f);
//                 glBegin(GL_LINES);
//                 glVertex3f(0, 0, 0);
//                 glVertex3f(w, h, z);
//                 glVertex3f(0, 0, 0);
//                 glVertex3f(w, -h, z);
//                 glVertex3f(0, 0, 0);
//                 glVertex3f(-w, -h, z);
//                 glVertex3f(0, 0, 0);
//                 glVertex3f(-w, h, z);
// 
//                 glVertex3f(w, h, z);
//                 glVertex3f(w, -h, z);
// 
//                 glVertex3f(-w, h, z);
//                 glVertex3f(-w, -h, z);
// 
//                 glVertex3f(-w, h, z);
//                 glVertex3f(w, h, z);
// 
//                 glVertex3f(-w, -h, z);
//                 glVertex3f(w, -h, z);
//                 glEnd();
// 
//                 glPopMatrix();
//             }
//         }
// 
//         if (bDrawGraph) {
//             glLineWidth(mGraphLineWidth);
//             glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
//             glBegin(GL_LINES);
// 
//             for (size_t i = 0; i < vpKFs.size(); i++) {
//                 // Covisibility Graph
//                 const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
//                 Vector3f Ow = vpKFs[i]->GetCameraCenter();
// 		Ow = vpKFs[i]->Tlc.cast<float>().block<3,3>(0,0) * Ow + vpKFs[i]->Tlc.cast<float>().block<3,1>(0,3);
//                 if (!vCovKFs.empty()) {
//                     for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
//                          vit != vend; vit++) {
//                         if ((*vit)->mnId < vpKFs[i]->mnId)
//                             continue;
//                         Vector3f Ow2 = (*vit)->GetCameraCenter();
// 			Ow2 = (*vit)->Tlc.cast<float>().block<3,3>(0,0) * Ow2+ (*vit)->Tlc.cast<float>().block<3,1>(0,3);
//                         glVertex3f(Ow[0], Ow[1], Ow[2]);
//                         glVertex3f(Ow2[0], Ow2[1], Ow2[2]);
//                     }
//                 }
// 
//                 // Spanning tree
//                 KeyFrame *pParent = vpKFs[i]->GetParent();
//                 if (pParent) {
//                     Vector3f Owp = pParent->GetCameraCenter();
// 		    Owp = pParent->Tlc.cast<float>().block<3,3>(0,0) * Owp+ pParent->Tlc.cast<float>().block<3,1>(0,3);
//                     glVertex3f(Ow[0], Ow[1], Ow[2]);
//                     glVertex3f(Owp[0], Owp[1], Owp[2]);
//                 }
// 
//                 // Loops
//                 set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
//                 for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
//                     if ((*sit)->mnId < vpKFs[i]->mnId)
//                         continue;
//                     Vector3f Owl = (*sit)->GetCameraCenter();
// 		    Owl = (*sit)->Tlc.cast<float>().block<3,3>(0,0) * Owl+ (*sit)->Tlc.cast<float>().block<3,1>(0,3);
//                     glVertex3f(Ow[0], Ow[1], Ow[2]);
//                     glVertex3f(Owl[0], Owl[1], Owl[2]);
//                 }
//             }
// 
//             glEnd();
//         }
//     }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();
	 pangolin::OpenGlMatrix Twc_ =/* mpLaserMap->Tlc * */Twc;

#ifdef HAVE_GLES
        glMultMatrixf(Twc_.m);
#else
        glMultMatrixd(Twc_.m);
#endif
	

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }

	void MapDrawer::DrawCurrentLaserPose(pangolin::OpenGlMatrix& Twl)
	{
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();
		pangolin::OpenGlMatrix Twl_ =  Twl;

#ifdef HAVE_GLES
        glMultMatrixf(Twl_.m);
#else
        glMultMatrixd(Twl_.m);
#endif
	

        glLineWidth(mCameraLineWidth);
        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
	}


    void MapDrawer::SetCurrentCameraPose(const SE3d &Tcw) {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw;
    }
    
	void MapDrawer::SetCurrentLaserPose(const Matrix4f& Tlw)
	{
		unique_lock<mutex> lock(mMutexLaser);
		mLaserPose = Tlw;
	}

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
        if (mCameraPose.log().norm() > 1e-12) {
            Matrix3d Rwc;
            Vector3d twc;
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rotationMatrix().transpose();
                twc = -1 * Rwc * mCameraPose.translation();
            }

            M.m[0] = Rwc(0, 0);
            M.m[1] = Rwc(1, 0);
            M.m[2] = Rwc(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc(0, 1);
            M.m[5] = Rwc(1, 1);
            M.m[6] = Rwc(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc(0, 2);
            M.m[9] = Rwc(1, 2);
            M.m[10] = Rwc(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc(0);
            M.m[13] = twc(1);
            M.m[14] = twc(2);
            M.m[15] = 1.0;
        } else
            M.SetIdentity();
    }
    
	void MapDrawer::GetCurrentOpenGLLaserMatrix(pangolin::OpenGlMatrix& M)
	{
		SE3d se3LaserPose(mLaserPose.inverse().cast<double>());
        if (se3LaserPose.log().norm() > 1e-12) {
            Matrix3d Rwc;
            Vector3d twc;
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = se3LaserPose.rotationMatrix().transpose();
                twc = -1 * Rwc * se3LaserPose.translation();
            }

            M.m[0] = Rwc(0, 0);
            M.m[1] = Rwc(1, 0);
            M.m[2] = Rwc(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc(0, 1);
            M.m[5] = Rwc(1, 1);
            M.m[6] = Rwc(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc(0, 2);
            M.m[9] = Rwc(1, 2);
            M.m[10] = Rwc(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc(0);
            M.m[13] = twc(1);
            M.m[14] = twc(2);
            M.m[15] = 1.0;
        } else
            M.SetIdentity();
	}


} //namespace vill
