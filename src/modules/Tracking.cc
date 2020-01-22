#include "Tracking.h"
#include "LocalMapping.h"
#include "Timer.h"
#include "System.h"
#include "FrameDrawer.h"
#include "ORBmatcher.h"
#include "MapDrawer.h"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include "glog/logging.h"

namespace vill {
	
	Tracking::Tracking(
			System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
			MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor,
			ConfigParam *pParams) :
			mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
			mpKeyFrameDB(pKFDB),  mpSystem(pSys), mpViewer(NULL),
			mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0) 
	{
				
		mpParams = pParams;

		mverbose = ConfigParam::_verbose;
		minInitFeaturePoints = ConfigParam::_minInitFeaturePoints;
		minInitMapPoints = ConfigParam::_minInitMapPoints;
		minInitTrackPoints = ConfigParam::_minInitTrackPoints;
		minTrackPoints = ConfigParam::_minTrackPoints;
		minLocalMapTrackPoints = ConfigParam::_minLocalMapTrackPoints;

		string tmpfilepath = ConfigParam::getTmpFilePath();
		frouteTrack.open(tmpfilepath + "routeTrack.txt");
		frouteTrack << std::fixed << std::setprecision(3);
		frtPose.open(tmpfilepath + "rtPose.txt");
		
		// Load camera parameters from settings file
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		float fx = fSettings["Camera.fx"];
		float fy = fSettings["Camera.fy"];
		float cx = fSettings["Camera.cx"];
		float cy = fSettings["Camera.cy"];
		mK = Matrix3f::Identity();
		mK(0, 0) = fx;
		mK(1, 1) = fy;
		mK(0, 2) = cx;
		mK(1, 2) = cy;

		int bUseDistK6 = fSettings["Camera.bUseDistK6"];
		if (bUseDistK6 == 1) {
			cv::Mat DistCoef(8, 1, CV_32F);
			DistCoef.at<float>(0) = fSettings["Camera.k1"];
			DistCoef.at<float>(1) = fSettings["Camera.k2"];
			DistCoef.at<float>(2) = fSettings["Camera.p1"];
			DistCoef.at<float>(3) = fSettings["Camera.p2"];
			DistCoef.at<float>(4) = fSettings["Camera.k3"];
			DistCoef.at<float>(5) = fSettings["Camera.k4"];
			DistCoef.at<float>(6) = fSettings["Camera.k5"];
			DistCoef.at<float>(7) = fSettings["Camera.k6"];
			DistCoef.copyTo(mDistCoef);

			for (size_t i = 0; i < 8; i++)
				if (DistCoef.at<float>(i) != 0)
					Frame::mbNeedUndistort = true;
		} else {
			cv::Mat DistCoef(4, 1, CV_32F);
			DistCoef.at<float>(0) = fSettings["Camera.k1"];
			DistCoef.at<float>(1) = fSettings["Camera.k2"];
			DistCoef.at<float>(2) = fSettings["Camera.p1"];
			DistCoef.at<float>(3) = fSettings["Camera.p2"];
			for (size_t i = 0; i < 4; i++)
				if (DistCoef.at<float>(i) != 0)
					Frame::mbNeedUndistort = true;
			const float k3 = fSettings["Camera.k3"];
			if (k3 != 0) {
				DistCoef.resize(5);
				DistCoef.at<float>(4) = k3;
				Frame::mbNeedUndistort = true;
			}
			DistCoef.copyTo(mDistCoef);
		}

		mbf = fSettings["Camera.bf"];

		float fps = fSettings["Camera.fps"];
		if (fps == 0)
			fps = 30;

		// Max/Min Frames to insert keyframes and to check relocalisation
		if (mSensor == System::RGBD)
			mMinFrames = 0;
		else
			mMinFrames = 0;
		mMaxFrames = fps;

		int nRGB = fSettings["Camera.RGB"];
		mbRGB = nRGB;
		
		// Load ORB parameters
		nFeatures = fSettings["ORBextractor.nFeatures"];
		fScaleFactor = fSettings["ORBextractor.scaleFactor"];
		nLevels = fSettings["ORBextractor.nLevels"];
		fIniThFAST = fSettings["ORBextractor.iniThFAST"];
		fMinThFAST = fSettings["ORBextractor.minThFAST"];

		mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
		if (sensor == System::STEREO)
			mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
		if (sensor == System::MONOCULAR)
			mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

		if (sensor == System::STEREO || sensor == System::RGBD) {
			mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
		}

		if (sensor == System::RGBD) {
			mDepthMapFactor = fSettings["DepthMapFactor"];
			if (fabs(mDepthMapFactor) < 1e-5)
				mDepthMapFactor = 1;
			else
				mDepthMapFactor = 1.0f / mDepthMapFactor;
		}

		//TEST deleted 
	// 	mpAlign = new vill::SparseImgAlign( nLevels - 1, 1);
		mbUseDirect = mpParams->_bUseDirect;
		mbUseIMU = mpParams->GetUseIMUFlag();
		mbUseLaserpose = mpParams->GetUseLaserPoseFlag();

		int nCacheHitTh = fSettings["Tracking.CacheFeatures"];
		if (nCacheHitTh) {
			mnCacheHitTh = nCacheHitTh;
		}
		
		lastRelocTime=0;
		mbRelocalizationSuccess = false;
		mbPubLoc = false;
		
		bDenseSfM = 0;
		bDenseSfM = fSettings["denseSfM"];
		
		printParams(frouteTrack);
	}
	
	
	Tracking::~Tracking() {
		if (mpAlign)
			delete mpAlign;
	}

	
	SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {
		mImGray = imRectLeft;
		cv::Mat imGrayRight = imRectRight;

		if (mImGray.channels() == 3) {
			if (mbRGB) {
				cv::cvtColor(mImGray, mImGray, CV_RGB2GRAY);
				cv::cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
// 				EqualizedImageLog(mImGray, mImGray);
// 				EqualizedImageLog(imGrayRight, imGrayRight);
			}
			else {
				cv::cvtColor(mImGray, mImGray, CV_BGR2GRAY);
				cv::cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
// 				EqualizedImageLog(mImGray, mImGray);
// 				EqualizedImageLog(imGrayRight, imGrayRight);
			}
		}
		else if (mImGray.channels() == 4) {
			if (mbRGB) {
				cv::cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
				cv::cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
			}
			else {
				cv::cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
				cv::cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
			}
		}

		mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
			mK, mDistCoef, mbf, mThDepth);
		mCurrentFrame.ExtractFeatures();

		Track();

		return mCurrentFrame.mTcw;
	}

	void Tracking::Track(){
		
		Timer tTrack;
		
		if (mState == NO_IMAGES_YET) {
			mState = NOT_INITIALIZED;
		}
		
		mLastProcessedState = mState;
		
		/// check whether the map is updated by localmapping
		bool bMapUpdated = CheckMapUpdated();
		
		if (mState == NOT_INITIALIZED) {
			if (mSensor == System::STEREO || mSensor == System::RGBD){ 
				
				StereoInitialization();
				
				
			}
			
			mpFrameDrawer->Update(this);
			
			if(mverbose && mState == OK)
				frouteTrack << "STATE: OK . Initilization SUC." << endl;
			if(mState != OK)
				return;
			
		} else {
			bool bOK;
			if( !mbOnlyTracking ){ //false
				
				if (mState == OK) {
					
					CheckReplacedInLastFrame();
					
					if ( mbVelocitySet == false) {
						bOK = TrackReferenceKeyFrame();
					} else {
						bOK = false;
						
						if (bOK == false) {
							bOK = TrackWithFeature(bMapUpdated);
						}
						
						if (!bOK) {
							bOK = TrackReferenceKeyFrame();
						}
					}
				} else { // mState != OK
					if(mbUseIMU == false) {
						// should relocalization
						
						LOG(ERROR) <<"TODO: relocalization";
						
						bOK = TrackReferenceKeyFrame();
						
						if(!bOK) TrackWithFeature(bMapUpdated);
					} else {
						LOG(ERROR) << "TODO: relocalization with IMU";
						return;
					}
				}
				
			} else {
				// could be moved to tracking failure ?
				bOK = routineOnlyTracking(bMapUpdated);
			}
			
			mCurrentFrame.mpReferenceKF = mpReferenceKF;
			
			if (!mbOnlyTracking) {
				
				if(bOK) {
					if( !mbUseIMU ){
						
						bOK = TrackLocalMap();
						
					}
				}
				
			} else {
				if (bOK && !mbVO) { bOK = TrackLocalMap();}
			}
			
			if (bOK) {
				mState = OK;
				mbVisionWeak = false;
			} else {

				mState = LOST;
				// without imu or imu not inited, set lost
				if (mverbose  && mState==OK)
					frouteTrack << "STATE: LOST. first time." << endl;
			
			}
			
			mpFrameDrawer->Update(this);
			
			if (bOK) {
				/// step 2.3 update mVelocity (@see TrackWithFeature)
				if (mLastFrame.mbPoseSet) {
					SE3f LastTwc = mLastFrame.mTcw.inverse();
					mVelocity = mCurrentFrame.mTcw * LastTwc;
					
					mbVelocitySet = true;
					if(mVelocity.translation().norm()>1){
						
						mbVelocitySet=false;
						
					}
				} else {
					mVelocity = SE3f();
					mbVelocitySet = false;
				}
				
				///drawing
				mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw.cast<double>());
				
				/// step2.4：clear some map points in current frame with little observation
				for (int i = 0; i < mCurrentFrame.N; i++) {
					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
					if (pMP) {
						if (pMP->Observations() < 1) {
							mCurrentFrame.mvbOutlier[i] = false;
							mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
						}
					}
				}
				
				/// step 2.5: delete points in mlpTemporalPoints 
				for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
						lit != lend; lit++) {
					MapPoint *pMP = *lit;
					delete pMP;
				}
				mlpTemporalPoints.clear();
				
				//detect new keyframe 
				if (NeedNewKeyFrame())
					CreateNewKeyFrame();
					
				// delete outliers
				for (int i = 0; i < mCurrentFrame.N; i++) {
					if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
						mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
				}
			} else {//bOK == false Track fail
				LOG(ERROR) << "Tracking failure";
			}
			
			
			/// step 2.7 tracking lost. If there is little keyframes in map, reset
			if (mState == LOST) {
				if (mpMap->KeyFramesInMap() <= 5)
					// if(!mpLocalMapper->GetVINSInited())
				{
					cout << "Track lost soon after initialisation, reseting..." << endl;
					mpSystem->Reset();
					return;
				}
			}
			
			
			if (!mCurrentFrame.mpReferenceKF)
				mCurrentFrame.mpReferenceKF = mpReferenceKF;
		
			if (mCurrentFrame.N > 30 || !mbUseDirect)
				mLastFrame = Frame(mCurrentFrame);
		}// end of !NOT_INITIALIZED
		
		CopyPose();
		
	}
	
	bool Tracking::TrackWithFeature(bool bTrackLastKF){
	// 	cout <<"[TrackWithFeature] tracked lastKF " <<bTrackLastKF<<endl;

		Timer tTrackMM;
		if (mCurrentFrame.mbFeatureExtracted == false)
			mCurrentFrame.ExtractFeatures();

		ORBmatcher matcher(0.9, true);

		UpdateLastFrame();


		if (!mbUseIMU ) {
			mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
		}
		else {
// 			PredictNavStateByIMU(bTrackLastKF);
			std::cout << "have not integrated IMU yet" << std::endl;
		}

		fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

		// Project points seen in previous frame
		int th = 7;
		if (mSensor != System::STEREO)
			th = 15;
		
		int tried = 0;
		int nmatches = matcher.SearchByProjection(tried, mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);
		if (nmatches < minTrackPoints + 10) {
			fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
			nmatches = matcher.SearchByProjection(tried, mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
		}

		if (nmatches < minTrackPoints + 10) {
			frouteTrack << "TrackWithFeature: matched: " << nmatches << " tried: " << tried << " cost: " << -1 << " [ms]." << endl;
			return false;
		}

		// Optimize frame pose with all matches
		//Optimizer::PoseOptimization(&mCurrentFrame);
		PoseOpt(&mCurrentFrame);

		// remove outlier mappoints：mCurrentFrame.mvpMapPoints[i] =NULL.
		// test valid MapPoint matches: nmatchesMap.
		int nmatchesMap = 0;
		for (int i = 0; i < mCurrentFrame.N; i++) {
			if (mCurrentFrame.mvpMapPoints[i]) {
				if (mCurrentFrame.mvbOutlier[i]) {
					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

					mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
					mCurrentFrame.mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = mCurrentFrame.mnId;
					nmatches--;
				}
				else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
					nmatchesMap++;
			}
		}

		float tTMM = tTrackMM.elapsed();
		
		mCurrentFrame.nmatches = nmatches;
		
		if (mverbose)
			frouteTrack << "TrackWithFeature: matched: " << nmatchesMap<<" tried: "<< tried<< " cost: "<< tTMM << " [ms]." << endl;
		if (mbOnlyTracking) {
			mbVO = nmatchesMap < 10;
			return nmatches > minTrackPoints + 10;
		}
		// TODO: nmatches VS nmatchesMap
		return nmatchesMap >= minTrackPoints;
	}


	/**
	* @brief track with the MapPoints in the mpReferenceKF
	* 
	* 1. SearchByBoW currentFrame & mpReferenceKF
	* 2. PoseOpt
	* @return if nmatches > 10, return true
	*/
	bool Tracking::TrackReferenceKeyFrame() {
		Timer tTrackRKF;
		cout <<"[Track ReferenceKF]" << endl;

		//syywh deleted
		// should not exist
		// 	if (mCurrentFrame.mbFeatureExtracted == false)
		// 		mCurrentFrame.ExtractFeatures();

		vector<MapPoint *> vpMapPointMatches;
		ORBmatcher matcher(0.7, false);
		int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);
		if (nmatches < 15) {
			frouteTrack << "TrackReferenceKeyFrame: "<< "nmatches[Map]: "<<nmatches<<" cost: " << -1 << " [ms]." << endl;
			return false;
		}
		mCurrentFrame.mvpMapPoints = vpMapPointMatches;
	
	// 	UpdateLastFramePose();
		mCurrentFrame.SetPose(mLastFrame.mTcw);

		/// pose optimization
		PoseOpt(&mCurrentFrame);
// 		if (mbUseIMU && mpLocalMapper->GetVINSInited()) {
// 		}
		
		// outlier removement
		int nmatchesMap = 0;
		for (int i = 0; i < mCurrentFrame.N; i++) {
			if (mCurrentFrame.mvpMapPoints[i]) {
				if (mCurrentFrame.mvbOutlier[i]) {
					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

					mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
					mCurrentFrame.mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = mCurrentFrame.mnId;
					nmatches--;
				}
				// TODO: check whether computed or not
				else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
					nmatchesMap++;
			}
		}
		float tTRKF = tTrackRKF.elapsed();
		if (mverbose)
			frouteTrack << "TrackReferenceKeyFrame: " << "nmatches[Map]: " << nmatchesMap << " cost: " << tTRKF << " [ms]." << endl;
		return nmatchesMap >= minTrackPoints;
	}	
	
	/**
	* @brief Stereo Initialization
	* 
	*/
	void Tracking::StereoInitialization() {

		if (mCurrentFrame.N > 500) {
			// Set Frame pose to the origin
	// 		mCurrentFrame.SetPose(SE3f());
			mCurrentFrame.SetPose(ConfigParam::GetSE3Tbc().inverse().cast<float>());


			
// 			KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, vimu1, NULL);
			KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
			pKFini->ComputeBoW(); //TODO whether keep or not
			
			// Insert KeyFrame in the map
			mpMap->AddKeyFrame(pKFini);

			// Create MapPoints and asscoiate to KeyFrame
			for (int i = 0; i < mCurrentFrame.N; i++) {
				float z = mCurrentFrame.mvDepth[i];
				if (z > 0) {
					Vector3f x3D = mCurrentFrame.UnprojectStereo(i);
					MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
					pNewMP->AddObservation(pKFini, i);
					pKFini->AddMapPoint(pNewMP, i);
					pNewMP->ComputeDistinctiveDescriptors();
					pNewMP->UpdateNormalAndDepth();
					mpMap->AddMapPoint(pNewMP);

					mCurrentFrame.mvpMapPoints[i] = pNewMP;
				}
			}

			LOG(INFO) << "[stereo initialization\n]New map created with " 
			<< mpMap->MapPointsInMap() << " points" << endl;

			mpLocalMapper->InsertKeyFrame(pKFini);

			mLastFrame = Frame(mCurrentFrame);
			mnLastKeyFrameId = mCurrentFrame.mnId;
			mpLastKeyFrame = pKFini;
			mLastFrame.mpReferenceKF = pKFini;


			//TEST syywh delete to remove loop detection
	// 		mvpLocalKeyFrames.push_back(pKFini);
			mvpLocalMapPoints = mpMap->GetAllMapPoints();
			mpReferenceKF = pKFini;
			mCurrentFrame.mpReferenceKF = pKFini;

			mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

			mpMap->mvpKeyFrameOrigins.push_back(pKFini);

			mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw.cast<double>());
			cout <<"stereo initialization with Tcw:\n"<<mCurrentFrame.mTcw.matrix()<<endl;


			mState = OK;
		}
	}
	
		
	/**
	* @brief Tracking with local map
	* 1. Search Local Map to expand matches
	* 2. PoseOpt
	* 3. Outlier removement
	* 
	*/
	bool Tracking::TrackLocalMap() {
		Timer tTrackLM;
		if (mvpLocalMapPoints.size() == 0)
			UpdateLocalMap();

	// 	if (mCurrentFrame.mbFeatureExtracted == false) {
	// 		mCurrentFrame.N = 0;
	// 		mCurrentFrame.ExtractFeatures();
	// 	}

		// Search points in Local Map which are in FOV of current frame
		SearchLocalPoints();

		// Optimize Pose
		PoseOpt(&mCurrentFrame);
		mnMatchesInliers = 0;

		for (int i = 0; i < mCurrentFrame.N; i++) {
			if (mCurrentFrame.mvpMapPoints[i]) {
				if (!mCurrentFrame.mvbOutlier[i]) {
					mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
					if (!mbOnlyTracking) {
						if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
							mnMatchesInliers++;
					}
					else
						mnMatchesInliers++;
				}
				else if (mSensor == System::STEREO)
					mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

			}
		}

		UpdateLocalMap();

		float tTLM = tTrackLM.elapsed();
		if (mverbose)
			frouteTrack << "TrackLocalMap: " <<"mnMatchesInliers: "<< mnMatchesInliers<< " "<< tTLM << " [ms]." << endl;
		
		if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 20+ minLocalMapTrackPoints) 
			return false;
		if (mnMatchesInliers < minLocalMapTrackPoints) 
			return false;
		return true;
	}


	/**
	* not used
	*/
	bool Tracking::routineOnlyTracking(bool bMapUpdated){
		bool bOK;
		
		
		if (mState == LOST || mState == NOT_INITIALIZED_LOC) {
			bOK = Relocalization_PnP();
			if(bOK){
				setRelocalizationState(true);
				setPubLoc(true);
			}
			else{
				setRelocalizationState(false);
				setPubLoc(false);
			}
		}
		else {
			if (!mbVO) {

				// In last frame we tracked enough MapPoints in the map
				if (mbVelocitySet) {
					bOK = TrackWithFeature( bMapUpdated);
	// 				bOK = TrackWithFeature(false || bMapUpdated);
	// 				bOK = TrackWithSparseAlignment(mpLocalMapper->GetFirstVINSInited() || bMapUpdated);
	// 				if(mCurrentFrame.mTimeStamp-lastRelocTime>5)
	// 					bOK = Relocalization();
				}
				else {
					bOK = TrackReferenceKeyFrame();
				}
			}
			else {
				// In last frame we tracked mainly "visual odometry" points.

				bool bOKMM = false;
				bool bOKReloc = false;
				vector<MapPoint *> vpMPsMM;
				vector<bool> vbOutMM;
				SE3f TcwMM;
				if (mbVelocitySet) {
					bOKMM = TrackWithFeature( bMapUpdated);
	// 				bOKMM = TrackWithFeature(false || bMapUpdated);
					vpMPsMM = mCurrentFrame.mvpMapPoints;
					vbOutMM = mCurrentFrame.mvbOutlier;
					TcwMM = mCurrentFrame.mTcw;
				}
				if(getRelocalizationState())
					bOKReloc = Relocalization_PnP();
				else
					bOKReloc = Relocalization_PnP();
				if(bOKReloc){
					setRelocalizationState(true);
					setPubLoc(true);
				}
				else{
					setRelocalizationState(false);
					setPubLoc(false);
				}

				if (bOKMM && !bOKReloc) {
					mCurrentFrame.SetPose(TcwMM);
					mCurrentFrame.mvpMapPoints = vpMPsMM;
					mCurrentFrame.mvbOutlier = vbOutMM;

					if (mbVO) {
						for (int i = 0; i < mCurrentFrame.N; i++) {
							if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
								mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
							}
						}
					}
				}
				else if (bOKReloc) {
					mbVO = false;
				}
				bOK = bOKReloc || bOKMM;
			}
		}
		
	// 	cout <<"endofroute"<<endl<<mCurrentFrame.mTcw.matrix()<<endl;
		

		
		if(mCurrentFrame.mTimeStamp-lastRelocTime > 2)
			mbVO = true;
		
		return bOK;
	}
	
}