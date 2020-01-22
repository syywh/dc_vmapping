#include "System.h"
//
#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

#include "MapDrawer.h"
#include "Viewer.h"
#include "FrameDrawer.h"

#include "KeyFrameDatabase.h"
#include "KeyFrame.h"
#include "Converter.h"

#include "MapPoint.h"
#include "Frame.h"

#include <glog/logging.h>
#include <iomanip>

#include "serialization.h"
namespace vill {

System::~System() {
	if (mpParams)
		delete mpParams;
}

bool System::GetLocalMapAcceptKF() {
	return (mpLocalMapper->AcceptKeyFrames() && !mpLocalMapper->isStopped()) ||
			mpLocalMapper->GetUpdatingInitPoses();
	//return mpLocalMapper->ForsyncCheckNewKeyFrames();
}

bool has_suffix(
		const std::string &str, const std::string &suffix) {
	std::size_t index = str.find(suffix, str.size() - suffix.size());
	return (index != std::string::npos);
}


System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
			const bool bUseViewer, ConfigParam *pParams): 
			mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), 
			mbActivateLocalizationMode(false),mbDeactivateLocalizationMode(false),
			trackingPoseUpdated(false)
{

	///Check settings file
	cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
	if (!fsSettings.isOpened()) {
		LOG(ERROR) << "Failed to open settings file at: " << strSettingsFile << endl;
		exit(-1);
	}

	mpParams = pParams;
	if (!mpParams)
		mpParams = new ConfigParam(strSettingsFile);

	fconfigSystem.open(ConfigParam::_tmpFilePath + "configSystem.txt");
	fconfigSystem <<"VISLAM trial." << endl;
	mpParams->printConfig(fconfigSystem);

	///Load ORB Vocabulary
	fconfigSystem << endl << "Loading ORB Vocabulary..." << endl;

	mpVocabulary = new ORBVocabulary();
	bool bVocLoad = false; // chose loading method based on file extension
	if (has_suffix(strVocFile, ".txt"))
		bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	else if (has_suffix(strVocFile, ".bin"))
		bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
	else
		bVocLoad = false;
	if (!bVocLoad) {
		fconfigSystem << "Wrong path to vocabulary. " << endl;
		fconfigSystem << "Falied to open at: " << strVocFile << endl;
		LOG(ERROR) << "EXIT. See " << ConfigParam::_tmpFilePath + "configSystem.txt for detailed info." << endl;
		exit(-1);
	}
	fconfigSystem << "Vocabulary loaded!" << endl << endl;

	/// thread construction.
	//Create KeyFrame Database
	mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

	//Create the Map
	mpMap = new Map();


	mTvelo_cam_inv_right = Matrix4f::Identity();


	//Create Drawers. These are used by the Viewer
	mpFrameDrawer = new FrameDrawer(mpMap);
	mpMapDrawer = new MapDrawer(mpMap, /*mpLaserMap,*/ strSettingsFile);

	//Initialize the Tracking thread
	//(it will live in the main thread of execution, the one that called this constructor)
	mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
							mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mpParams);
	mpTracker->printParams(fconfigSystem);

	//Initialize the Local Mapping thread and launch
	mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, /*mpLaserMap,*/ mpParams);
	mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);

	//TEST syywh delete the loop closing thread
	//Initialize the Loop Closing thread and launch
// 	mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, mpParams);
// 	mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

	//Initialize the Viewer thread and launch
	if (bUseViewer) {
		mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
		mptViewer = new thread(&Viewer::Run, mpViewer);
		mpTracker->SetViewer(mpViewer);
// 		mpLocalMapper->SetViewer(mpViewer);
	}

	//TEST syywh
	//Set pointers between threads
	mpTracker->SetLocalMapper(mpLocalMapper);
// 	mpTracker->SetLoopClosing(mpLoopCloser);

// 	mpTracker->SetLaserMap(mpLaserMap);
	mpLocalMapper->SetTracker(mpTracker);
// 	mpLocalMapper->SetLoopCloser(mpLoopCloser);

// 	mpLoopCloser->SetTracker(mpTracker);
// 	mpLoopCloser->SetLocalMapper(mpLocalMapper);
	
	

}




cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) {
	if (mSensor != STEREO) {
		cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
		exit(-1);
	}


	SE3f Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

	SetTrackingPoseUpdated(Tcw, timestamp);
	
	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = mpTracker->mState;
	mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
	return Converter::toCvMat(Tcw.matrix());
}


void System::ActivateLocalizationMode() {
	unique_lock<mutex> lock(mMutexMode);
	mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
	unique_lock<mutex> lock(mMutexMode);
	mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
	static int n = 0;
	int curn = mpMap->GetLastBigChangeIdx();
	if (n < curn) {
		n = curn;
		return true;
	} else
		return false;
}

void System::Reset() {
	unique_lock<mutex> lock(mMutexReset);
	mbReset = true;
}

void System::Shutdown() {
	mpLocalMapper->RequestFinish();
// 	mpLoopCloser->RequestFinish();
	if (mpViewer) {
		mpViewer->RequestFinish();
		while (!mpViewer->isFinished())
			std::this_thread::sleep_for(std::chrono::milliseconds(5));

	}

	// Wait until all thread have effectively stopped
	while (!mpLocalMapper->isFinished() /*|| !mpLoopCloser->isFinished() *//*|| mpLoopCloser->isRunningGBA()*/) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	}

	          
	if (mpViewer)
		pangolin::BindToContext("Vill: Map Viewer");
}

void System::SaveLocationResult(const string& filename)
{
	cout << endl << "Saving location trajectory to " << filename << " ..." << endl;

	std::map<double, std::pair<vector<float>, Vector3f>> mKFs = mpLocalMapper->mPoses;

	// Transform all keyframes so that the first keyframe is at the origin.
	// After a loop closure the first keyframe might not be at the origin.
	//cv::Mat Two = vpKFs[0]->GetPoseInverse();

	ofstream f;
	f.open(filename.c_str());
	f << fixed;

	for (auto it = mKFs.begin(), ite = mKFs.end(); it != ite; it++) {
		
		f << setprecision(6) << it->first << setprecision(7) << " " << it->second.second[0] << " " << it->second.second[1] << " " << it->second.second[2]
			<< " " << it->second.first[0] << " " << it->second.first[1] << " " << it->second.first[2] << " " << it->second.first[3] << endl;

	}

	f.close();
	cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryTUM(const string &filename) {
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    SE3f Two = vpKFs[0]->GetPoseInverseOri();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
//     LOG(INFO) << mpLaserMap->FirstOptimizedid  <<endl;
    list<vill::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;
// 		if(pKF->mnId < mpLaserMap->FirstOptimizedid) continue; //TODO
		if(pKF->mnId < 2) continue;//没进优化，四元数好像不太对
// 	LOG(INFO )<< pKF->mnId<<" "<<pKF->mnFrameId<<endl;

        SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;
		// Trw = pKF->GetPoseOri() * (SE3f(pKF->Tlc.cast<float>()).inverse()) ;//Tcw
		//Trw = Trw*pKF->GetPose();

        SE3f Tcw = (*lit)*Trw;
        Matrix<float, 3 , 3> Rwc = Tcw.rotationMatrix().inverse();
        Matrix<float, 3 , 1> twc = -Rwc*Tcw.translation();
		
		cv::Mat R = Converter::toCvMat(Tcw.rotationMatrix());
		vector<float> qq = Converter::toQuaternion(R);
// 		cout<<qq[0]<<" "<<qq[1]<<" "<<qq[2]<<" "<<qq[3]<<endl;;

		Eigen::Quaternionf q(Rwc);
        vector<float> vq(4);
        vq[0] = q.x();
        vq[1] = q.y();
        vq[2] = q.z();
        vq[3] = q.w();

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc(0,0) << " " << twc(1,0) << " " << twc(2,0) << " " << vq[0] << " " << vq[1] << " " << vq[2] << " " << vq[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
//     mpLocalMapper->file.close();
}


void System::SaveLoops(const string& filename)
{
	cout <<"Saving loops to " << filename << endl;
	map<pair<long unsigned int, long unsigned int>, cv::Mat>::iterator mit = mpLoopCloser->mLoops.begin();
	
	ofstream f;
	f.open(filename.c_str());
	f<<fixed;
	
	for(; mit != mpLoopCloser->mLoops.end(); mit++){
		cv::Mat relativePose = mit->second;
		
		f << mit->first.first<<" " << mit->first.second<< endl;
		f << relativePose << endl;
	}
	f.close();

}


int System::GetTrackingState() {
	unique_lock<mutex> lock(mMutexState);
	return mTrackingState;
}

vector<MapPoint *> System::GetTrackedMapPoints() {
	unique_lock<mutex> lock(mMutexState);
	return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
	unique_lock<mutex> lock(mMutexState);
	return mTrackedKeyPoints;
}

LocalMapping* System::getLocalMappingPtr(){
  return mpLocalMapper;
}

bool System::GetTrackingPoseUpdated(SE3f& pose, double& cTime)
{
	unique_lock<mutex> lock(mMutexTrackingPoseUpdated);
	if(trackingPoseUpdated){
		pose = currentPoseTcw.inverse();
		cTime = currentTime;
		trackingPoseUpdated = false;
		return true;
	}else
		return false;
}

void System::SetTrackingPoseUpdated(SE3f& currentpose,const double& cTime)
{
	unique_lock<mutex> lock(mMutexTrackingPoseUpdated);
	currentPoseTcw = currentpose;
	currentTime = cTime;
	trackingPoseUpdated = true;
}


void System::saveData(const string& filename)
{
	string filename_mappoint = filename + "/mappoints.cereal";
	string filename_keyframe = filename + "/keyframes.cereal";
	string filename_kftimeid = filename + "/keyframes_time_id.txt";
	
	std::ofstream of_mp(filename_mappoint);
	std::ofstream of_kf(filename_keyframe);
	
	std::ofstream ok_kf_timeid(filename_kftimeid);
		
	//save keyframes
	//add fen'ge'fu, like a string 
	int blank = 0;
	{
		cout <<"Saving binary KF to " << filename_keyframe << endl;
		cereal::BinaryOutputArchive arkf(of_kf);
		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		int kf_num = vpKFs.size();
		arkf(kf_num);
		for(size_t i = 0; i < vpKFs.size(); i++){
			KeyFrame* pKF = vpKFs[i];
			bool exist = true;
			if(!pKF) {
				exist = false;
				arkf(exist);
				continue;
			}
			
			ok_kf_timeid << to_string(pKF->mTimeStamp) <<" " << pKF->mnId << endl;
		
			arkf(exist);
			arkf(pKF->mnId,pKF->mnFrameId ,pKF->cx, pKF->cy, pKF->fx, pKF->fy, 
				pKF->mb, pKF->mbf, pKF->invfx, pKF->invfy,pKF->mThDepth,pKF->isBad(),
				(float)(pKF->mnMinX), (float)(pKF->mnMinY), 
				 (float)(pKF->mnMaxX), (float)(pKF->mnMaxY),
				pKF->mfGridElementWidthInv,pKF->mfGridElementHeightInv,
				pKF->mTimeStamp,pKF->N, 
				pKF->mnScaleLevels, pKF->mfScaleFactor,pKF->mfLogScaleFactor
				);
			
			

			
			arkf(pKF->mvDepth);	
			arkf(pKF->mDescriptors);	//cv::Mat	
			arkf(pKF->mvScaleFactors);	//vector<float>		
			arkf(' ');		//vector<cv::KeyPoint>
			arkf(pKF->mvLevelSigma2);	//vector<float>
			arkf(pKF->mvKeys);			//vector<cv::KeyPoint>
			arkf(pKF->mvuRight);		//vector<float>
			map<unsigned int, double> reBowVec(pKF->mBowVec.begin(), pKF->mBowVec.end());
					//vector<float>
			arkf(reBowVec);			//map<unsigned int, double>
			arkf(pKF->mvInvLevelSigma2);//vector<float>
			map<unsigned int, vector<unsigned int> > reFeatVec(pKF->mFeatVec.begin(), pKF->mFeatVec.end());
			
			arkf(reFeatVec);		//map<unsigned int, vector<int> >
			
			arkf(pKF->mK);				//cv::Mat
			arkf(pKF->mGrid);			//vector<vector<vector<size_t> > >
			Eigen::Matrix4f pose(pKF->GetPose().matrix());
			arkf(pose);		//cv::Mat
			
			//mvpOrderedConnectedKeyFrames
			vector<KeyFrame*> pKF_mvpOrderedConnectedKFs = pKF->GetVectorCovisibleKeyFrames();
			vector<int> re_pKF_mvpOrderedConnectedKFs;
			for(size_t j = 0; j < pKF_mvpOrderedConnectedKFs.size(); j++){
				re_pKF_mvpOrderedConnectedKFs.push_back(pKF_mvpOrderedConnectedKFs[j]->mnId);
			}
			arkf(re_pKF_mvpOrderedConnectedKFs);	//vector<int>
			//mConnectedKeyFrameWeights
			arkf(pKF->getConnectedKeyFrameWeights());	//map<int,int>
			vector<MapPoint*> vMapPoints = pKF->GetMapPointMatches();
			vector<int> re_vMapPoints;
			for(size_t j = 0; j < vMapPoints.size(); j++)	{		
				if(!(vMapPoints[j])) {
					re_vMapPoints.push_back(-1);
					continue;
				}
				re_vMapPoints.push_back(vMapPoints[j]->mnId);
			}
			arkf(re_vMapPoints);					//vector<int>

		}
		
		of_kf.close();
	}
	
	{
		//save mappoints
		cout <<"Saving binary MP to " << filename_mappoint << endl;
		cereal::BinaryOutputArchive armp(of_mp);
		vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
		
		std::ofstream mpFile(filename + "/mappoints.txt");
		
		int mappoint_num = vpMPs.size();
		armp(mappoint_num);
		
		vector<KeyFrame*> pKFs = mpMap->GetAllKeyFrames();
		KeyFrame* pKF_first = pKFs[0];
		Eigen::Matrix4f pose(pKF_first->GetPose().matrix());
		for(size_t i = 0; i < vpMPs.size(); i++){
			MapPoint* pMP = vpMPs[i];
			bool exist = true;
			if(!pMP){
				exist  = false;
				armp(exist);
				continue;			 	
			}
			
			
// 			Vector3f p = pMP->GetWorldPos();
// 			Vector3f point_in_c = pose.block<3,3>(0,0) * p + pose.block<3,1>(0,3);
// 			mpFile << point_in_c.transpose() << endl;
			
			armp(exist);
			armp(pMP->mnId, pMP->mnFirstKFid, pMP->nObs);
			
			armp(pMP->GetWorldPos());	//cv::Mat
			armp(pMP->isBad());
			armp(pMP->GetNormal());		//Eigen::Vector3f
			armp(pMP->getmfMinDistance());
			armp(pMP->GetDescriptor());	//cv::Mat
			armp(pMP->getmfMaxDistance());
			
			
			//mObservations
			map<KeyFrame*, size_t> mObs = pMP->GetObservations();
			map<int, int> re_mObs;
			map<KeyFrame*, size_t>::iterator mObs_it = mObs.begin();
			for(; mObs_it != mObs.end(); mObs_it++){
				re_mObs[mObs_it->first->mnId] = mObs_it->second;
			}
			armp(re_mObs);
			armp(pMP->GetReferenceKeyFrame()->mnId);
		}
		of_mp.close();
	
	}
	
	//save lines
	{
		vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		
		string filename_image =filename+"/images.txt";
		string filename_point3d =filename+"/points3D.txt";
		string filename_tinestamp =filename+"/timestamp.txt";
		string filename_cameras = filename+"/cameras.txt";
		string picPath = filename+"/images/";
		
		boost::filesystem::path dir(picPath);
		boost::filesystem::create_directory(dir);
	

        ofstream f_image;
        f_image.open(filename_image.c_str());
        f_image << fixed;

		ofstream f_point3d;
        f_point3d.open(filename_point3d.c_str());
        f_point3d << fixed;

		ofstream f_timestamp;
        f_timestamp.open(filename_tinestamp.c_str());
        f_timestamp << fixed;
		
// 	# Camera list with one line of data per camera:
// #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
// # Number of cameras: 1
// 1 PINHOLE 648 488 402.6082767683831 402.6082767683831 303.936897277832 247.9286670684814
		ofstream f_cameras;
		f_cameras.open(filename_cameras.c_str());
		f_cameras << fixed;
		f_cameras << "# Camera list with one line of data per camera:\n";
		f_cameras << "# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
		f_cameras << "# Number of cameras: 1\n";
		
		int camID=1;
		if(vpKFs.size()>0)
		{
			KeyFrame *pKF = vpKFs[0];
			cv::Mat img = pKF->mvImagePyramid[0];
			int img_width = img.cols;
			int img_height = img.rows;
			f_cameras << camID << " PINHOLE " << img_width << " " << img_height << " " << pKF->fx
			<< " " << pKF->fy << " " << pKF->cx << " " << pKF->cy << std::endl;
		}

		for (size_t j = 0; j < vpKFs.size(); j++) {
            KeyFrame *pKF = vpKFs[j];

			cv::Mat R = Converter::toCvMat(pKF->GetRotation());
            vector<float> q = Converter::toQuaternion(R);
			char picName[100];
			sprintf(picName,"%010d.png",pKF->mnId);
			// save img
			cv::Mat img = pKF->mvImagePyramid[0];
			string savepicName;
			savepicName = picPath+picName;
// 			cv::imwrite(savepicName,img);
		
            Vector3f t = pKF->GetTranslation();
			f_image << pKF->mnId << " " <<setprecision(7)<<q[3] <<" "<<q[0] << " " << q[1] << " " << q[2] << " " << t[0] << " " << t[1] << " " << t[2]

			  <<" 1 "<< setprecision(6) << picName<<endl;
            if (pKF->isBad())
                continue;
	    
			const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();
	    
			set<MapPoint *>::iterator it;
			for(it=spAlreadyFound.begin();it!=spAlreadyFound.end();it++) 
			{
				MapPoint *pMP = *it;
				map<KeyFrame *, size_t> observations = pMP->GetObservations();
				for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
						mit != mend; mit++)
				{
					if (mit->first->mnId == pKF->mnId)
					{
					
						size_t obsid=mit->second;
						Vector3f pos1 = pMP->GetWorldPos();
						Vector3f pos =pos1-pKF->GetCameraCenter();
						Vector2f proj;
						proj[0]=pos[0]/pos[2];
						proj[1]=pos[1]/pos[2];

						Vector2f res;
						res[0] = proj[0] * pKF->fx + pKF->cx;
						res[1] = proj[1] * pKF->fy + pKF->cy;
						
						Vector2f Obs;
						Obs[0]=pKF->mvKeys[obsid].pt.x;
						Obs[1]=pKF->mvKeys[obsid].pt.y;
						
						Vector2f errorgubo;
						errorgubo=Obs-res;
						float cost_squarednorm=errorgubo.squaredNorm(); //求二范数;
						
						f_image << pKF->mvKeys[obsid].pt.x <<" "<<pKF->mvKeys[obsid].pt.y<<" "<<pMP->mnId;
						f_image << " ";
					}
				
				}
			}
			f_image << " "<<endl;
			f_timestamp << pKF->mnId << " " <<pKF->mnFrameId<<" "<< setprecision(6) << pKF->mTimeStamp<<" " <<setprecision(7)<<q[3] <<" "<<q[0] << " " << q[1] << " " << q[2] 

			<< " " << t[0] << " " << t[1] << " " << t[2] << endl;
	    
        }
	     
		const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
	  
            if (vpMPs[i]->isBad())
                continue;

			MapPoint *pMP2 = vpMPs[i];
	    
			Vector3f pos = pMP2->GetWorldPos();
			map<KeyFrame *, size_t> observations = vpMPs[i]->GetObservations();
			f_point3d <<pMP2->mnId<<" "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" 255 0 0"<<" ";
			bool bdonotSave = false;
	    
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
	      
				for (size_t j = 0; j < vpKFs.size(); j++) 
				{
					KeyFrame *pKF = vpKFs[j];

					if (pKF->isBad())
						continue;
					if (mit->first->mnId == pKF->mnId)
					{
						cv::Mat R = Converter::toCvMat(pKF->GetRotation()).t();
						vector<float> q = Converter::toQuaternion(R);
						Vector3f t = pKF->GetCameraCenter();
						Vector3f pos = vpMPs[i]->GetWorldPos();
					
						size_t obsid=mit->second;
						Vector2f proj;
						proj[0]=pos[0]/pos[2];
						proj[1]=pos[1]/pos[2];
						Vector2f res;
						res[0] = proj[0] * pKF->fx + pKF->cx;
						res[1] = proj[1] * pKF->fy + pKF->cy;
					
						Vector2f Obs;
						Obs[0]=pKF->mvKeys[obsid].pt.x;
						Obs[1]=pKF->mvKeys[obsid].pt.y;
						Vector2f errorgubo;
						errorgubo=Obs-res;
						float cost_squarednorm=errorgubo.squaredNorm(); //求二范数;
					
						if(bdonotSave == false)

						{
							f_point3d <<cost_squarednorm<<" ";
							bdonotSave = true;
						}
						f_point3d << pKF->mnId<<" "<<obsid<<" ";

					}
				}
			}
			f_point3d << " "<<endl;
		}

		f_image.close();
		f_point3d.close();
		f_timestamp.close();
		f_cameras.close();

        cout << endl << "data saved!" << endl;
		
	}
	

}

void System::saveDataKFMP(const string& filename)
{
	string filename_mappoint = filename + "/mappoints.cereal";
	string filename_keyframe = filename + "/keyframes.cereal";
	
	std::ofstream of_mp(filename_mappoint);
	std::ofstream of_kf(filename_keyframe);
		
	//save keyframes
	//add fen'ge'fu, like a string 
	int blank = 0;
	{
		cout <<"Saving binary KF to " << filename_keyframe << endl;
		cereal::BinaryOutputArchive arkf(of_kf);
		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		int kf_num = vpKFs.size();
		arkf(kf_num);
		for(size_t i = 0; i < vpKFs.size(); i++){
			KeyFrame* pKF = vpKFs[i];
			bool exist = true;
			if(!pKF) {
				exist = false;
				arkf(exist);
				continue;
			}
			

		
			arkf(exist);
			arkf(pKF->mnId,pKF->mnFrameId ,pKF->cx, pKF->cy, pKF->fx, pKF->fy, 
				pKF->mb, pKF->mbf, pKF->invfx, pKF->invfy,pKF->mThDepth,pKF->isBad(),
				(float)(pKF->mnMinX), (float)(pKF->mnMinY), 
				 (float)(pKF->mnMaxX), (float)(pKF->mnMaxY),
				pKF->mfGridElementWidthInv,pKF->mfGridElementHeightInv,
				pKF->mTimeStamp,pKF->N, 
				pKF->mnScaleLevels, pKF->mfScaleFactor,pKF->mfLogScaleFactor
				);
			


			
			arkf(pKF->mvDepth);	
			arkf(pKF->mDescriptors);	//cv::Mat	
			arkf(pKF->mvScaleFactors);	//vector<float>		
			arkf(' ');		//vector<cv::KeyPoint>
			arkf(pKF->mvLevelSigma2);	//vector<float>
			arkf(pKF->mvKeys);			//vector<cv::KeyPoint>
			arkf(pKF->mvuRight);		//vector<float>
			map<unsigned int, double> reBowVec(pKF->mBowVec.begin(), pKF->mBowVec.end());
					//vector<float>
			arkf(reBowVec);			//map<unsigned int, double>
			arkf(pKF->mvInvLevelSigma2);//vector<float>
			map<unsigned int, vector<unsigned int> > reFeatVec(pKF->mFeatVec.begin(), pKF->mFeatVec.end());

			arkf(reFeatVec);		//map<unsigned int, vector<int> >
			
			arkf(pKF->mK);				//cv::Mat
			arkf(pKF->mGrid);			//vector<vector<vector<size_t> > >
			Eigen::Matrix4f pose(pKF->GetPose().matrix());
			arkf(pose);		//cv::Mat

			//mvpOrderedConnectedKeyFrames
			vector<KeyFrame*> pKF_mvpOrderedConnectedKFs = pKF->GetVectorCovisibleKeyFrames();

			vector<int> re_pKF_mvpOrderedConnectedKFs;
			for(size_t j = 0; j < pKF_mvpOrderedConnectedKFs.size(); j++){
				if(pKF_mvpOrderedConnectedKFs[j]){
					re_pKF_mvpOrderedConnectedKFs.push_back(pKF_mvpOrderedConnectedKFs[j]->mnId);
				}
			}
			arkf(re_pKF_mvpOrderedConnectedKFs);	//vector<int>
			

			//mConnectedKeyFrameWeights
			arkf(pKF->getConnectedKeyFrameWeights());	//map<int,int>
			vector<MapPoint*> vMapPoints = pKF->GetMapPointMatches();
			vector<int> re_vMapPoints;
			for(size_t j = 0; j < vMapPoints.size(); j++)	{		
				if(!(vMapPoints[j])) {
					re_vMapPoints.push_back(-1);
					continue;
				}
				re_vMapPoints.push_back(vMapPoints[j]->mnId);
			}
			arkf(re_vMapPoints);					//vector<int>


		}
		
		of_kf.close();
	}
	
	{
		//save mappoints
		cout <<"Saving binary MP to " << filename_mappoint << endl;
		cereal::BinaryOutputArchive armp(of_mp);
		vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
		int mappoint_num = vpMPs.size();
		armp(mappoint_num);
		for(size_t i = 0; i < vpMPs.size(); i++){
			MapPoint* pMP = vpMPs[i];
			bool exist = true;
			if((!pMP)||(!(pMP->GetReferenceKeyFrame()))||(pMP->GetReferenceKeyFrame()->isBad())){
				exist  = false;
				armp(exist);
				continue;				
			}

			armp(exist);
			armp(pMP->mnId, pMP->mnFirstKFid, pMP->nObs);
			

			
			armp(pMP->GetWorldPos());	//cv::Mat
			armp(pMP->isBad());
			armp(pMP->GetNormal());		//Eigen::Vector3f
			armp(pMP->getmfMinDistance());
			armp(pMP->GetDescriptor());	//cv::Mat
			armp(pMP->getmfMaxDistance());
			

			
			//mObservations
			map<KeyFrame*, size_t> mObs = pMP->GetObservations();
			map<int, int> re_mObs;
			map<KeyFrame*, size_t>::iterator mObs_it = mObs.begin();
			for(; mObs_it != mObs.end(); mObs_it++){
				if(mObs_it->first != nullptr)
					re_mObs[mObs_it->first->mnId] = mObs_it->second;
			}
			
			armp(re_mObs);
			if(pMP->GetReferenceKeyFrame())
				armp(pMP->GetReferenceKeyFrame()->mnId);
			else
				armp(-1);

		}
		of_mp.close();
	
	}
	
	std::cout<<"save cereal done" << std::endl;
	
	
	{
		
		vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		
		string filename_image =filename+"/images.txt";
		string filename_point3d =filename+"/points3D.txt";
		string filename_tinestamp =filename+"/timestamp.txt";
		string filename_cameras = filename+"/cameras.txt";
		
		ofstream f_image;
        f_image.open(filename_image.c_str());
        f_image << fixed;

		ofstream f_point3d;
        f_point3d.open(filename_point3d.c_str());
        f_point3d << fixed;

		ofstream f_timestamp;
        f_timestamp.open(filename_tinestamp.c_str());
        f_timestamp << fixed;
		
		ofstream f_cameras;
		f_cameras.open(filename_cameras.c_str());
		f_cameras << fixed;
		
		cout <<"saving lines" << endl;
		f_cameras << "# Camera list with one line of data per camera:\n";
		f_cameras << "# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
		f_cameras << "# Number of cameras: 1\n";
		
		int camID=1;
		if(vpKFs.size()>0)
		{
			KeyFrame *pKF = vpKFs[0];
// 			cv::Mat img = ConfigParam::;
// 			int img_width = img.cols;
// 			int img_height = img.rows;
			f_cameras << camID << " PINHOLE " << mpParams->cols_ << " " << mpParams->rows_ << " " << pKF->fx
			<< " " << pKF->fy << " " << pKF->cx << " " << pKF->cy << std::endl;
		}
		
		
		for (size_t j = 0; j < vpKFs.size(); j++) {
            KeyFrame *pKF = vpKFs[j];

			cv::Mat R = Converter::toCvMat(pKF->GetRotation());
            vector<float> q = Converter::toQuaternion(R);
			char picName[100];
			sprintf(picName,"%010d.png",pKF->mnId);
// 			// save img
// 			cv::Mat img = pKF->mvImagePyramid[0];
// 			string savepicName;
// 			savepicName = picPath+picName;
// 			cv::imwrite(savepicName,img);
		
            Vector3f t = pKF->GetTranslation();
			f_image << pKF->mnId << " " <<setprecision(7)<<q[3] <<" "<<q[0] << " " << q[1] << " " << q[2] << " " << t[0] << " " << t[1] << " " << t[2]

			  <<" 1 "<< setprecision(6) << picName<<endl;
            if (pKF->isBad())
                continue;
	    
			const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();
	    
			set<MapPoint *>::iterator it;
			for(it=spAlreadyFound.begin();it!=spAlreadyFound.end();it++) 
			{
				MapPoint *pMP = *it;
				map<KeyFrame *, size_t> observations = pMP->GetObservations();
				for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
						mit != mend; mit++)
				{
					if (mit->first->mnId == pKF->mnId)
					{
					
						size_t obsid=mit->second;
						Vector3f pos1 = pMP->GetWorldPos();
						Vector3f pos =pos1-pKF->GetCameraCenter();
						Vector2f proj;
						proj[0]=pos[0]/pos[2];
						proj[1]=pos[1]/pos[2];

						Vector2f res;
						res[0] = proj[0] * pKF->fx + pKF->cx;
						res[1] = proj[1] * pKF->fy + pKF->cy;
						
						Vector2f Obs;
						Obs[0]=pKF->mvKeys[obsid].pt.x;
						Obs[1]=pKF->mvKeys[obsid].pt.y;
						
						Vector2f errorgubo;
						errorgubo=Obs-res;
						float cost_squarednorm=errorgubo.squaredNorm(); //求二范数;
						
						f_image << pKF->mvKeys[obsid].pt.x <<" "<<pKF->mvKeys[obsid].pt.y<<" "<<pMP->mnId;
						f_image << " ";
					}
				
				}
			}
			f_image << " "<<endl;
			f_timestamp << pKF->mnId << " " <<pKF->mnFrameId<<" "<< setprecision(6) << pKF->mTimeStamp<<" " <<setprecision(7)<<q[3] <<" "<<q[0] << " " << q[1] << " " << q[2] 

			<< " " << t[0] << " " << t[1] << " " << t[2] << endl;
	    
        }
	     
		const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
	  
            if (vpMPs[i]->isBad())
                continue;

			MapPoint *pMP2 = vpMPs[i];
	    
			Vector3f pos = pMP2->GetWorldPos();
			map<KeyFrame *, size_t> observations = vpMPs[i]->GetObservations();
			f_point3d <<pMP2->mnId<<" "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" 255 0 0"<<" ";
			bool bdonotSave = false;
	    
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
	      
				for (size_t j = 0; j < vpKFs.size(); j++) 
				{
					KeyFrame *pKF = vpKFs[j];

					if (pKF->isBad())
						continue;
					if (mit->first->mnId == pKF->mnId)
					{
						cv::Mat R = Converter::toCvMat(pKF->GetRotation()).t();
						vector<float> q = Converter::toQuaternion(R);
						Vector3f t = pKF->GetCameraCenter();
						Vector3f pos = vpMPs[i]->GetWorldPos();
					
						size_t obsid=mit->second;
						Vector2f proj;
						proj[0]=pos[0]/pos[2];
						proj[1]=pos[1]/pos[2];
						Vector2f res;
						res[0] = proj[0] * pKF->fx + pKF->cx;
						res[1] = proj[1] * pKF->fy + pKF->cy;
					
						Vector2f Obs;
						Obs[0]=pKF->mvKeys[obsid].pt.x;
						Obs[1]=pKF->mvKeys[obsid].pt.y;
						Vector2f errorgubo;
						errorgubo=Obs-res;
						float cost_squarednorm=errorgubo.squaredNorm(); //求二范数;
					
						if(bdonotSave == false)

						{
							f_point3d <<cost_squarednorm<<" ";
							bdonotSave = true;
						}
						f_point3d << pKF->mnId<<" "<<obsid<<" ";

					}
				}
			}
			f_point3d << " "<<endl;
		}

		f_image.close();
		f_point3d.close();
		f_timestamp.close();
		f_cameras.close();

        cout << endl << "data saved!" << endl;
	}
}

std::map<int, cv::Mat> System::getDepthMap(){
	
	vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	std::cout <<"keyframes size " << vpKFs.size() <<std::endl;
	
	Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
	K(0,0) = vpKFs[0]->fx;
	K(1,1) = vpKFs[0]->fy;
	K(0,2) = vpKFs[0]->cx;
	K(1,2) = vpKFs[0]->cy;
	
	std::map<int, cv::Mat> m_id_depth;
	
	for(size_t i = 0; i < vpKFs.size(); i++){
		vill::KeyFrame* pKF = vpKFs[i];
		if(pKF->isBad()) continue;
		
		Eigen::Matrix4f pose(pKF->GetPose().matrix());
		double fx, fy, cx, cy;
		int rows = pKF->mvImagePyramid[0].rows;
		int cols = pKF->mvImagePyramid[0].cols;
		
		
		cv::Mat im = cv::Mat(rows, cols, CV_16U, cv::Scalar(0));
		
		//init gradient map
		pKF->gradient_for_mp = cv::Mat(rows, cols, CV_16U, cv::Scalar(0));
		
		int cnt = 0;
// 		vector<MapPoint*> vMapPoints = pKF->GetMapPointMatches();
		vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
		

		for(size_t j = 0; j < vMapPoints.size(); j++)	{	
			MapPoint* pMp = vMapPoints[j];
			if((!pMp) || pMp->isBad()) continue;
			
			Vector3f p = pMp->GetWorldPos();
			Vector3f point_in_c = pose.block<3,3>(0,0) * p + pose.block<3,1>(0,3);
			
			if(point_in_c[2]<=0.5) continue;
			
			Vector3f proj(point_in_c[0]/point_in_c[2], point_in_c[1]/point_in_c[2],1);
			
			proj = K * proj;
			
			Vector2i res;
			res[0] = (int)(proj[0]);
			res[1] = (int)(proj[1]);
// 						cout << res[0] <<" " << res[1] <<endl;
			if(res[1]<0 || res[1] >= rows || res[0] <0 || res[0]>=cols || point_in_c[2]>40 || point_in_c[2]<=0) continue;
			

			
			im.at<int>(res[1], res[0]) = point_in_c[2]*256;
			
			pKF->m_pt_mpid[pair<int,int>(res[1],res[0])] = pMp->mnId;
			cnt++;
		}
		
		m_id_depth[pKF->mnId] = im;

	}
	
	return m_id_depth;
	
}


void System::saveDepthMap(const string& filepath)
{
	string file_folder = filepath + "/depth_maps";
	boost::filesystem::path dir(file_folder.c_str());
	boost::filesystem::create_directory(dir);
	
	std::cout << "Saving depth file to "<< file_folder<<std::endl;
	vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	std::cout <<"keyframes size " << vpKFs.size() <<std::endl;
	
	Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
	K(0,0) = vpKFs[0]->fx;
	K(1,1) = vpKFs[0]->fy;
	K(0,2) = vpKFs[0]->cx;
	K(1,2) = vpKFs[0]->cy;
	
	for(size_t i = 0; i < vpKFs.size(); i++){
		vill::KeyFrame* pKF = vpKFs[i];
		if(pKF->isBad()) continue;
		
		Eigen::Matrix4f pose(pKF->GetPose().matrix());
		double fx, fy, cx, cy;
		int rows = pKF->mvImagePyramid[0].rows;
		int cols = pKF->mvImagePyramid[0].cols;
		
		cout<<"saving keyframe " << pKF->mnId <<" with ";
		
		cv::Mat im = cv::Mat(rows, cols, CV_16U, cv::Scalar(0));
		
		int cnt = 0;
// 		vector<MapPoint*> vMapPoints = pKF->GetMapPointMatches();
		vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
		
		ofstream pMPf(filepath + "/depth_maps/"+to_string(pKF->mnId)+".txt");
		
		for(size_t j = 0; j < vMapPoints.size(); j++)	{	
			MapPoint* pMp = vMapPoints[j];
			if((!pMp) || pMp->isBad()) continue;
			
			Vector3f p = pMp->GetWorldPos();
			Vector3f point_in_c = pose.block<3,3>(0,0) * p + pose.block<3,1>(0,3);
			
			if(point_in_c[2]<=0.5) continue;
			
			Vector3f proj(point_in_c[0]/point_in_c[2], point_in_c[1]/point_in_c[2],1);
			
			proj = K * proj;
			
			Vector2i res;
			res[0] = (int)(proj[0]);
			res[1] = (int)(proj[1]);
// 						cout << res[0] <<" " << res[1] <<endl;
			if(res[1]<0 || res[1] >= rows || res[0] <0 || res[0]>=cols || point_in_c[2]>40 || point_in_c[2]<=0) continue;
			
// 			if((im.at<int>(res[1], res[0])>0) && (im.at<int>(res[1], res[0])<point_in_c[2])) continue;
			
			pMPf << point_in_c.transpose() <<" "<<pKF->fx <<" "<<pKF->fy<<" "<<pKF->cx<<" "<<pKF->cy << endl;
			
			im.at<int>(res[1], res[0]) = point_in_c[2]*256;
			cnt++;
		}
		
		pMPf.close();
		cout << cnt<<" points" << endl;
		char picName[1000];
		sprintf(picName,"%010d.png",pKF->mnId);
		string imname = file_folder+"/"+picName;
		cv::imwrite(imname, im );
	}
}




bool System::loadData(const string& strDataFile)
{
	string filename_keyframe = strDataFile + "/keyframes.cereal";
	string filename_mappoint = strDataFile + "/mappoints.cereal";
	
	std::ifstream if_kf(filename_keyframe);
	std::ifstream if_mp(filename_mappoint);
	
	//load KeyFrame
	{
		cout << "Load KeyFrame from " <<filename_keyframe << endl;
		cereal::BinaryInputArchive arkf(if_kf);
		int kf_num;
		arkf(kf_num);
		cout <<"keyframe number is " << kf_num << endl;
		
		Frame* pF = new Frame();
		pF->mpORBvocabulary = mpVocabulary;
		
		for(int i = 0; i < kf_num; i++){
			
			bool exist;
			arkf(exist);

			if(!exist) continue;
			
			long unsigned int mnId =0;	bool isbad;
			arkf(mnId,pF->mnId ,pF->cx, pF->cy, pF->fx, pF->fy, 
				pF->mb, pF->mbf, pF->invfx, pF->invfy,pF->mThDepth,isbad,
				pF->mnMinX, pF->mnMinY, pF->mnMaxX, pF->mnMaxY,
				pF->mfGridElementWidthInv,pF->mfGridElementHeightInv,
				 pF->mTimeStamp,pF->N,
				pF->mnScaleLevels, pF->mfScaleFactor,pF->mfLogScaleFactor);
			
			
			pF->mvDepth = vector<float>(pF->N,-1.0f);
			arkf(pF->mvDepth);
			arkf(pF->mDescriptors);	//cv::Mat		
			arkf(pF->mvScaleFactors);	//vector<float>	
			char atemp;
			arkf(atemp);		//vector<cv::KeyPoint>
			arkf(pF->mvLevelSigma2);	//vector<float>
			arkf(pF->mvKeys);			//vector<cv::KeyPoint>
			arkf(pF->mvuRight);		
			

			map<unsigned int, double> reBowVec;
			
			arkf(reBowVec);	

			pF->mBowVec.clear();
			pF->mBowVec.insert(reBowVec.begin(), reBowVec.end());;
			
			arkf(pF->mvInvLevelSigma2);

// 			
			map<unsigned int, vector<unsigned int> > reFeatVec;
			arkf(reFeatVec);
			pF->mFeatVec.clear();
			pF->mFeatVec.insert(reFeatVec.begin(), reFeatVec.end());
			arkf(pF->mK);
			std::vector< std::vector <std::vector<size_t> > > mGrid;
			arkf(mGrid);

			Eigen::Matrix4f Tcw;
			arkf(Tcw);
			
			KeyFrame* pKF = new KeyFrame(mpMap, *pF);
			if(isbad)
				pKF->SetBadFlag();
			pKF->mnId = mnId;
			pKF->mGrid = mGrid;
			SE3f se3Tcw(Tcw);
			pKF->SetPose(se3Tcw);

			arkf(pKF->re_mvpOrderedConnectedKFs);
			arkf(pKF->mConnectedKF_weights);
			arkf(pKF->re_vMapPoints);

			
			mpMap->AddKeyFrame(pKF);
		}
		if_kf.close();
	}
// 	
	{
		cout << "Load MapPoint from " <<filename_mappoint << endl;
		cereal::BinaryInputArchive armp(if_mp);
		int mp_num;
		armp(mp_num);
		cout <<"mappoint number is " << mp_num << endl;
		
		for(int i = 0; i < mp_num; i++){
			MapPoint* pMP = new MapPoint(mpMap);
			
			bool exist;
			armp(exist);
			if(!exist) continue;
			
			armp(pMP->mnId, pMP->mnFirstKFid, pMP->nObs);
			
			
			Eigen::Vector3f worldPos;
			armp(worldPos);	//cv::Mat

			pMP->SetWorldPos(worldPos);
			bool isbad;
			armp(isbad);
			if(isbad)
				pMP->SetBadFlag();
			Eigen::Vector3f normal;
			armp(normal);		//cv::Mat
			pMP->setNormal(normal);
			float dist;
			armp(dist);
			pMP->setmfMinDistance(dist);
			cv::Mat desc;
			armp(desc);
			pMP->setDescriptor(desc);
			armp(dist);	//cv::Mat
			
			pMP->setmfMaxDistance(dist);			
			armp(pMP->re_mObs);
			armp(pMP->refKFid);
			
			
			mpMap->AddMapPoint(pMP);
		}
	}
	
	cout <<"load data done, recover..." << endl;
	//recover connection
	vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
	map<int, KeyFrame*> mId_KeyFrame;
	for(size_t i = 0; i < vKeyFrames.size(); i++){
		KeyFrame* pKF = vKeyFrames[i];
		mId_KeyFrame[pKF->mnId] = pKF;
	
	}
	
	
	vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
	map<int, MapPoint*> mId_MapPoint;
	for(size_t i = 0; i < vMapPoints.size(); i++){
		MapPoint* pMP = vMapPoints[i];
		mId_MapPoint[pMP->mnId] = pMP;
	}
	

	for(size_t i = 0; i < vKeyFrames.size(); i++){
		KeyFrame* pKF = vKeyFrames[i];
		for(size_t j = 0; j < pKF->re_mvpOrderedConnectedKFs.size(); j++){
			if(mId_KeyFrame.find(pKF->re_mvpOrderedConnectedKFs[j])!=mId_KeyFrame.end())
				pKF->addmvpOrderedConnectedKFs(mId_KeyFrame[pKF->re_mvpOrderedConnectedKFs[j]]);
		}
		
		map<int, int>::iterator cKF_weight_it = pKF->mConnectedKF_weights.begin();
		for(; cKF_weight_it != pKF->mConnectedKF_weights.end(); cKF_weight_it++){
			pKF->addmConnectedKF_weights(mId_KeyFrame[cKF_weight_it->first], cKF_weight_it->second);
		}
		
		for(size_t j = 0; j < pKF->re_vMapPoints.size(); j++){
			if(pKF->re_vMapPoints[j] == -1)
				pKF->addMapPoint(static_cast<MapPoint*>(NULL));
			else
				pKF->addMapPoint(mId_MapPoint[pKF->re_vMapPoints[j]]);
		}
		
		pKF->UpdateBestCovisibles();

	}
	

	
	for(size_t i = 0; i < vMapPoints.size(); i++){
		MapPoint* pMP = vMapPoints[i];
		
		map<int, int>::iterator re_obs_it = pMP->re_mObs.begin();

		for(; re_obs_it != pMP->re_mObs.end(); re_obs_it++){
			if((mId_KeyFrame.find(re_obs_it->first) != mId_KeyFrame.end())){
				
				pMP->AddObservation(mId_KeyFrame[re_obs_it->first], re_obs_it->second);
			}
			else{
				cout <<"what? " << re_obs_it->first<< endl;
			}
		}
		if(mId_KeyFrame.find(pMP->refKFid) != mId_KeyFrame.end()){
			pMP->setRefKF(mId_KeyFrame[pMP->refKFid]);
		}else{
			cout <<"do not have RefKF?" << endl;
		}
	}

	//database
	for(size_t i = 0; i < vKeyFrames.size(); i++){
		mpKeyFrameDatabase->add(vKeyFrames[i]);
	}
	
	return true;

}

bool System::loadData(const string& strDataFile, bool load_line, bool load_mapimg)
{
	string filename_keyframe = strDataFile + "/keyframes.cereal";
	string filename_mappoint = strDataFile + "/mappoints.cereal";
	string filename_mapline,filename_linedesc;
	if(load_line){
		filename_mapline = strDataFile + "/Line3D.txt";
		filename_linedesc = strDataFile + "/Line3Ddesc.txt";
	}
	
	std::ifstream if_kf(filename_keyframe);
	std::ifstream if_mp(filename_mappoint);
// 	cout << "line path is " << filename_mapline << endl;
	
	//load KeyFrame
	{
		cout << "Load KeyFrame from " <<filename_keyframe << endl;
		cereal::BinaryInputArchive arkf(if_kf);
		int kf_num;
		arkf(kf_num);
		cout <<"keyframe number is " << kf_num << endl;
		
		Frame* pF = new Frame();
		pF->mpORBvocabulary = mpVocabulary;
		
		for(int i = 0; i < kf_num; i++){
			
			bool exist;
			arkf(exist);

			if(!exist) continue;
			
			long unsigned int mnId =0;	bool isbad;
			arkf(mnId,pF->mnId ,pF->cx, pF->cy, pF->fx, pF->fy, 
				pF->mb, pF->mbf, pF->invfx, pF->invfy,pF->mThDepth,isbad,
				pF->mnMinX, pF->mnMinY, pF->mnMaxX, pF->mnMaxY,
				pF->mfGridElementWidthInv,pF->mfGridElementHeightInv,
				 pF->mTimeStamp,pF->N,
				pF->mnScaleLevels, pF->mfScaleFactor,pF->mfLogScaleFactor);
			
			
			pF->mvDepth = vector<float>(pF->N,-1.0f);
			arkf(pF->mvDepth);
			arkf(pF->mDescriptors);	//cv::Mat		
			arkf(pF->mvScaleFactors);	//vector<float>	
			char atemp;
			arkf(atemp);		//vector<cv::KeyPoint>
			arkf(pF->mvLevelSigma2);	//vector<float>
			arkf(pF->mvKeys);			//vector<cv::KeyPoint>
			arkf(pF->mvuRight);		
			

			map<unsigned int, double> reBowVec;
			
			arkf(reBowVec);	

			pF->mBowVec.clear();
			pF->mBowVec.insert(reBowVec.begin(), reBowVec.end());;
			
			arkf(pF->mvInvLevelSigma2);

// 			
			map<unsigned int, vector<unsigned int> > reFeatVec;
			arkf(reFeatVec);
			pF->mFeatVec.clear();
			pF->mFeatVec.insert(reFeatVec.begin(), reFeatVec.end());
			arkf(pF->mK);
			std::vector< std::vector <std::vector<size_t> > > mGrid;
			arkf(mGrid);

			Eigen::Matrix4f Tcw;
			arkf(Tcw);
			
			KeyFrame* pKF = new KeyFrame(mpMap, *pF);
			if(isbad)
				pKF->SetBadFlag();
			pKF->mnId = mnId;
			pKF->mGrid = mGrid;
			SE3f se3Tcw(Tcw);
			pKF->SetPose(se3Tcw);

			arkf(pKF->re_mvpOrderedConnectedKFs);
			arkf(pKF->mConnectedKF_weights);
			arkf(pKF->re_vMapPoints);

			
			mpMap->AddKeyFrame(pKF);
		}
		if_kf.close();
	}
// 	
	{
		cout << "Load MapPoint from " <<filename_mappoint << endl;
		cereal::BinaryInputArchive armp(if_mp);
		int mp_num;
		armp(mp_num);
		cout <<"mappoint number is " << mp_num << endl;
		
		for(int i = 0; i < mp_num; i++){
			MapPoint* pMP = new MapPoint(mpMap);
			
			bool exist;
			armp(exist);
			if(!exist) continue;
			
			armp(pMP->mnId, pMP->mnFirstKFid, pMP->nObs);
			
// 			std::cout << pMP->mnId <<std::endl;
			
			Eigen::Vector3f worldPos;
			armp(worldPos);	//cv::Mat

			pMP->SetWorldPos(worldPos);
			bool isbad;
			armp(isbad);
			if(isbad)
				pMP->SetBadFlag();
			Eigen::Vector3f normal;
			armp(normal);		//cv::Mat
			pMP->setNormal(normal);
			float dist;
			armp(dist);
			pMP->setmfMinDistance(dist);
			cv::Mat desc;
			armp(desc);
			pMP->setDescriptor(desc);
			armp(dist);	//cv::Mat
			pMP->setmfMaxDistance(dist);			
			armp(pMP->re_mObs);
			armp(pMP->refKFid);
			
			mpMap->AddMapPoint(pMP);
		}
	}
	
	cout <<"load data done, recover..." << endl;
	//recover connection
	vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
	map<int, KeyFrame*> mId_KeyFrame;
	string picPath = mapimgPath+"/images/";
	for(size_t i = 0; i < vKeyFrames.size(); i++){
		KeyFrame* pKF = vKeyFrames[i];
		
		mId_KeyFrame[pKF->mnId] = pKF;
		// save image to keyframe 
		if(load_mapimg){
			char picName[100];
			sprintf(picName,"%s%010d.png",picPath.c_str(),pKF->mnId);
			cv::Mat img = cv::imread(picName,cv::IMREAD_GRAYSCALE);
			if(!img.data)
			{
				std::cout << "cannot load img " << picName << endl;
			}
			pKF->AddKeyFrameImage(img);
			
		}
	}
	
	if(load_line){
		std::ifstream if_ml(filename_mapline);
		std::ifstream if_ld(filename_linedesc);
		cout <<"load line and recover line..." << endl;
		//load line connection
		std::string ml_line;
		while(std::getline(if_ml,ml_line))
		{}

	}
	
	vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
	map<int, MapPoint*> mId_MapPoint;
	for(size_t i = 0; i < vMapPoints.size(); i++){
		MapPoint* pMP = vMapPoints[i];
		mId_MapPoint[pMP->mnId] = pMP;
	}
	

	for(size_t i = 0; i < vKeyFrames.size(); i++){
		KeyFrame* pKF = vKeyFrames[i];
		for(size_t j = 0; j < pKF->re_mvpOrderedConnectedKFs.size(); j++){
			pKF->addmvpOrderedConnectedKFs(mId_KeyFrame[pKF->re_mvpOrderedConnectedKFs[j]]);
		}
		
		map<int, int>::iterator cKF_weight_it = pKF->mConnectedKF_weights.begin();
		for(; cKF_weight_it != pKF->mConnectedKF_weights.end(); cKF_weight_it++){
			pKF->addmConnectedKF_weights(mId_KeyFrame[cKF_weight_it->first], cKF_weight_it->second);
		}
		
		for(size_t j = 0; j < pKF->re_vMapPoints.size(); j++){
			if(pKF->re_vMapPoints[j] == -1)
				pKF->addMapPoint(static_cast<MapPoint*>(NULL));
			else
				pKF->addMapPoint(mId_MapPoint[pKF->re_vMapPoints[j]]);
		}
		
		pKF->UpdateBestCovisibles();

	}
	

	
	for(size_t i = 0; i < vMapPoints.size(); i++){
		MapPoint* pMP = vMapPoints[i];
		
		map<int, int>::iterator re_obs_it = pMP->re_mObs.begin();

		for(; re_obs_it != pMP->re_mObs.end(); re_obs_it++){
			if((mId_KeyFrame.find(re_obs_it->first) != mId_KeyFrame.end())){
				
				pMP->AddObservation(mId_KeyFrame[re_obs_it->first], re_obs_it->second);
			}
			else{
				cout <<"what? " << re_obs_it->first<< endl;
			}
		}
		
		if(mId_KeyFrame.find(pMP->refKFid) != mId_KeyFrame.end()){
			pMP->setRefKF(mId_KeyFrame[pMP->refKFid]);
		}else{
			cout <<"do not have RefKF?" << endl;
		}
	}

	//database
	for(size_t i = 0; i < vKeyFrames.size(); i++){
		mpKeyFrameDatabase->add(vKeyFrames[i]);
	}
	
	return true;

}

Tracking* System::getTrackingPtr()
{
	return mpTracker;
}


//fb savetotxt

void System::SaveToTXT(const string &folderName) {
	cout << endl << "Saving data to " << folderName << " ..." << endl;


	vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
	sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		
	string filename_image =folderName+"/images.txt";
	string filename_point3d =folderName+"/points3D.txt";
	string filename_tinestamp =folderName+"/timestamp.txt";
	string filename_cameras = folderName+"/cameras.txt";
	string picPath = folderName+"/images/";
	
    boost::filesystem::path dir(picPath);
    boost::filesystem::create_directory(dir);
	

	ofstream f_image;
	f_image.open(filename_image.c_str());
	f_image << fixed;

	ofstream f_point3d;
	f_point3d.open(filename_point3d.c_str());
	f_point3d << fixed;

	ofstream f_timestamp;
	f_timestamp.open(filename_tinestamp.c_str());
	f_timestamp << fixed;
	
// 	# Camera list with one line of data per camera:
// #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
// # Number of cameras: 1
// 1 PINHOLE 648 488 402.6082767683831 402.6082767683831 303.936897277832 247.9286670684814
	ofstream f_cameras;
	f_cameras.open(filename_cameras.c_str());
	f_cameras << fixed;
	f_cameras << "# Camera list with one line of data per camera:\n";
	f_cameras << "# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
	f_cameras << "# Number of cameras: 1\n";
	
	int camID=1;
	if(vpKFs.size()>0)
	{
		KeyFrame *pKF = vpKFs[0];
		cv::Mat img = pKF->mvImagePyramid[0];
		int img_width = img.cols;
		int img_height = img.rows;
		f_cameras << camID << " PINHOLE " << img_width << " " << img_height << " " << pKF->fx
		<< " " << pKF->fy << " " << pKF->cx << " " << pKF->cy << std::endl;
	}

	for (size_t j = 0; j < vpKFs.size(); j++) {
		KeyFrame *pKF = vpKFs[j];

		cv::Mat R = Converter::toCvMat(pKF->GetRotation());
		vector<float> q = Converter::toQuaternion(R);
		char picName[100];
		sprintf(picName,"%010d.png",pKF->mnId);
		// save img
		cv::Mat img = pKF->mvImagePyramid[0];
		string savepicName;
		savepicName = picPath+picName;
		cv::imwrite(savepicName,img);
	
		Vector3f t = pKF->GetTranslation();
		f_image << pKF->mnId << " " <<setprecision(7)<<q[3] <<" "<<q[0] << " " << q[1] << " " << q[2] << " " << t[0] << " " << t[1] << " " << t[2]

			<<" 1 "<< setprecision(6) << picName<<endl;
		if (pKF->isBad())
			continue;
	
		const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();
	    
	    set<MapPoint *>::iterator it;
	    for(it=spAlreadyFound.begin();it!=spAlreadyFound.end();it++) 
	    {
	                  MapPoint *pMP = *it;
	      	       map<KeyFrame *, size_t> observations = pMP->GetObservations();
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++)
		 {
		     if (mit->first->mnId == pKF->mnId)
	     {
           
	    size_t obsid=mit->second;
	      Vector3f pos1 = pMP->GetWorldPos();
	      Vector3f pos =pos1-pKF->GetCameraCenter();
	      Vector2f proj;
	      proj[0]=pos[0]/pos[2];
	      proj[1]=pos[1]/pos[2];

            Vector2f res;
            res[0] = proj[0] * pKF->fx + pKF->cx;
            res[1] = proj[1] * pKF->fy + pKF->cy;
	    
	    Vector2f Obs;
	    Obs[0]=pKF->mvKeys[obsid].pt.x;
	    Obs[1]=pKF->mvKeys[obsid].pt.y;
	    
	    Vector2f errorgubo;
	    errorgubo=Obs-res;
	    float cost_squarednorm=errorgubo.squaredNorm(); //求二范数;
	      
           f_image << pKF->mvKeys[obsid].pt.x <<" "<<pKF->mvKeys[obsid].pt.y<<" "<<pMP->mnId;
	              f_image << " ";
	     }
		   
		}
	    }
		 f_image << " "<<endl;
	     f_timestamp << pKF->mnId << " " <<pKF->mnFrameId<<" "<< setprecision(6) << pKF->mTimeStamp<<" " <<setprecision(7)<<q[3] <<" "<<q[0] << " " << q[1] << " " << q[2] 

	    << " " << t[0] << " " << t[1] << " " << t[2] << endl;
	    
        }
	     
	 const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
	  
            if (vpMPs[i]->isBad())
                continue;

	    MapPoint *pMP2 = vpMPs[i];
	    
	      Vector3f pos = pMP2->GetWorldPos();
	       map<KeyFrame *, size_t> observations = vpMPs[i]->GetObservations();
	     f_point3d <<pMP2->mnId<<" "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" 255 0 0"<<" ";
	    bool bdonotSave = false;
	    
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
	      
	         for (size_t j = 0; j < vpKFs.size(); j++) {
            KeyFrame *pKF = vpKFs[j];

            if (pKF->isBad())
                continue;
	     if (mit->first->mnId == pKF->mnId)
	     {
            cv::Mat R = Converter::toCvMat(pKF->GetRotation()).t();
            vector<float> q = Converter::toQuaternion(R);
            Vector3f t = pKF->GetCameraCenter();
			Vector3f pos = vpMPs[i]->GetWorldPos();
	      
			size_t obsid=mit->second;
			Vector2f proj;
			proj[0]=pos[0]/pos[2];
			proj[1]=pos[1]/pos[2];
            Vector2f res;
            res[0] = proj[0] * pKF->fx + pKF->cx;
            res[1] = proj[1] * pKF->fy + pKF->cy;
	    
			Vector2f Obs;
			Obs[0]=pKF->mvKeys[obsid].pt.x;
			Obs[1]=pKF->mvKeys[obsid].pt.y;
			Vector2f errorgubo;
			errorgubo=Obs-res;
			float cost_squarednorm=errorgubo.squaredNorm(); //求二范数;
	    
	    if(bdonotSave == false)

		{
		f_point3d <<cost_squarednorm<<" ";
		bdonotSave = true;
		}
		f_point3d << pKF->mnId<<" "<<obsid<<" ";

	     }
        }
		}
    f_point3d << " "<<endl;
        }

    f_image.close();
	f_point3d.close();
	f_timestamp.close();
	f_cameras.close();

        cout << endl << "data saved!" << endl;

    }
    
 
	bool System::CheckLocalMappingState()
	{
		return mpLocalMapper->AcceptKeyFrames();

	}
	
	void System::update_gradient(map< int, Mat >& m_id_gradient, map<int, float>& m_id_loss)
	{
		vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
		
		for(size_t i = 0; i < vKeyFrames.size(); i++){
			vill::KeyFrame* pKF = vKeyFrames[i];
			if(pKF->isBad()) continue;
			
			if(m_id_gradient.find(pKF->mnId) != m_id_gradient.end()){
				pKF->gradient_for_mp = m_id_gradient[pKF->mnId];
				pKF->loss = m_id_loss[pKF->mnId];
			}
		}

	}




} //namespace ORB_SLAM
