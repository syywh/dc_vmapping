#include "Tracking.h"
#include "LocalMapping.h"
#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"
#include "ORBmatcher.h"
#include "Optimizer_g2o.h"
#include "System.h"
#include "Timer.h"
#include "KeyFrameDatabase.h"
#include "PnPsolver.h"
#include "Converter_g2o.h"

#include "glog/logging.h"

namespace vill {

	/**
	* @brief check is there any map points in the last frame replaced
	*
	* some points might be replaced in local mapping
	* for the usage of mLastFrame，detect the replaced map points
	* @see LocalMapping::SearchInNeighbors()
	*/
	void Tracking::CheckReplacedInLastFrame() {
		for (int i = 0; i < mLastFrame.N; i++) {
			MapPoint *pMP = mLastFrame.mvpMapPoints[i];

			if (pMP) {
				MapPoint *pRep = pMP->GetReplaced();
				if (pRep) {
					mLastFrame.mvpMapPoints[i] = pRep;
				}
			}
		}
		

	}
	
	
	void Tracking::CopyPose(void){

		int tmpweak = 0;
		if (mbVisionWeak)
			tmpweak = 1;
	// 	frtPose << mCurrentFrame.mnId << " " << mState << " " << tmpweak << " " << mCurrentFrame.mpReferenceKF->mnFrameId << endl;
	// 	frtPose << mCurrentFrame.mTcw.matrix() << endl;
	// 	if(mCurrentFrame.mpReferenceKF->GetInitializedByVIO())
	// 	{
	// 		cout << "udpate last kf, cur is\n "<< mCurrentFrame.mTcw.matrix() <<endl;
	// 		SE3f newTcw = mCurrentFrame.mTcw * (ConfigParam::GetSE3Tbc().inverse()).cast<float>() * mpLocalMapper->getInitialTWi().inverse();
	// 		mCurrentFrame.SetPose(newTcw);
	// 		SE3d newTwi = (ConfigParam::GetSE3Tbc()* newTcw.cast<double>()).inverse();
	// 		NavState nsl = mLastFrame.GetNavState();
	// 		Vector3d oriV = nsl.Get_V();
	// 		nsl.Set_Pos(newTwi.translation());
	// 		nsl.Set_Rot(newTwi.so3());
	// 		nsl.Set_Vel(mpLocalMapper->getInitialTWi().cast<double>() * ConfigParam::GetSE3Tbc() * oriV);
	// 		mCurrentFrame.SetNavState(nsl);
	// 		mCurrentFrame.mpReferenceKF->ResetInitializedByVIO();
	// 		cout << "udpate last kf, cur is\n "<< mCurrentFrame.mTcw.matrix() <<endl;
	// 	}
		if (mCurrentFrame.mbPoseSet) {
			// compute relative T_currentFrame_referenceKeyFrame

			SE3f Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();		
			mlRelativeFramePoses.push_back(Tcr);
			mlpReferences.push_back(mpReferenceKF);
			mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
			mlbLost.push_back(mState == LOST);
		}
		else {
			// This can happen if tracking is lost
			mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
			mlpReferences.push_back(mlpReferences.back());
			mlFrameTimes.push_back(mlFrameTimes.back());
			mlbLost.push_back(mState == LOST);
		}
	}
	
	/**
	* @brief update the pose of last frame\n
	* 		 add more map points observed in the last frame
	*
	*/
	void Tracking::UpdateLastFrame() {
		// Update pose according to reference keyframe
		// cout<<"calling update last frame"<<endl;
		KeyFrame *pRef = mLastFrame.mpReferenceKF;
		SE3f Tlr = mlRelativeFramePoses.back();
		
		mLastFrame.SetPose(Tlr * pRef->GetPose());


		if (mnLastKeyFrameId == mLastFrame.mnId ||  !mbOnlyTracking)
			return;

		// Create "visual odometry" MapPoints
		// We sort points according to their measured depth by the stereo/RGB-D sensor
		vector<pair<float, int> > vDepthIdx;
		vDepthIdx.reserve(mLastFrame.N);
		for (int i = 0; i < mLastFrame.N; i++) {
			float z = mLastFrame.mvDepth[i];
			if (z > 0) {
				vDepthIdx.push_back(make_pair(z, i));
			}
		}

		if (vDepthIdx.empty())
			return;

		sort(vDepthIdx.begin(), vDepthIdx.end());

		// We insert all close points (depth<mThDepth)
		// If less than 100 close points, we insert the 100 closest ones.
		int nPoints = 0;
		for (size_t j = 0; j < vDepthIdx.size(); j++) {
			int i = vDepthIdx[j].second;

			bool bCreateNew = false;

			MapPoint *pMP = mLastFrame.mvpMapPoints[i];
			if (!pMP)
				bCreateNew = true;
			else if (pMP->Observations() < 1) {
				bCreateNew = true;
			}

			if (bCreateNew) {
				Vector3f x3D = mLastFrame.UnprojectStereo(i);
				MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

				mLastFrame.mvpMapPoints[i] = pNewMP;

				mlpTemporalPoints.push_back(pNewMP);
				nPoints++;
			}
			else {
				nPoints++;
			}

			if (vDepthIdx[j].first > mThDepth && nPoints > 100)
				break;
		}
	}
	
	int Tracking::PoseOpt(Frame *pFrame){
// 		return Optimizer::PoseOptimization(pFrame);
// 		if(ConfigParam::GetUseCeres())
// 			return Optimizer_ceres::PoseOptimizationCeres(pFrame);
// 		else
			return Optimizer_g2o::PoseOptimization(pFrame);
	}
	
	
	
	bool Tracking::NeedNewKeyFrame() {

		if (mbOnlyTracking)
			return false;

		// While updating initial poses
		if (mpLocalMapper->GetUpdatingInitPoses()) {
			LOG(WARNING) << "mpLocalMapper->GetUpdatingInitPoses, no new KF" << endl;
			return false;
		}

		// If Local Mapping is freezed by a Loop Closure do not insert keyframes
		if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
			return false;

		const int nKFs = mpMap->KeyFramesInMap();
		
		if(bDenseSfM) return true;

		if( !mbUseIMU )
		{
			// Do not insert keyframes if not enough frames have passed from last relocalisation
			// mMaxFrames: the rate of images
			if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames && (!mbUseIMU || mpLocalMapper->GetVINSInited()))
			return false;

			// Tracked MapPoints in the reference keyframe
			int nMinObs = 3;
			if (nKFs <= 2)
			nMinObs = 2;
			int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

			// Local Mapping accept keyframes?
			bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

			// Check how many "close" points are being tracked and how many could be potentially created.
			// Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
			// "total matches = matches to map + visual odometry matches"
			// Visual odometry matches will become MapPoints if we insert a keyframe.
			// This ratio measures how many MapPoints we could create if we insert a keyframe.
			int nNonTrackedClose = 0;
			int nTrackedClose = 0;
			if (mSensor != System::MONOCULAR) {
			for (int i = 0; i < mCurrentFrame.N; i++) {
				if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
				if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
					nTrackedClose++;
				else
					nNonTrackedClose++;
				}
			}
			}

			bool bNeedToInsertClose = 0;        
	// 		bNeedToInsertClose = (nTrackedClose < /*100*/60) && (nNonTrackedClose > /*70*/190);//comment for outdoor

			// Thresholds
			// decision
			float thRefRatio = 0.75f;
			if (nKFs < 2)
			thRefRatio = 0.4f;

			if (mSensor == System::MONOCULAR)
			thRefRatio = 0.9f;

			double timegap = 0.5, largetimegap = 5.0;  //0.2  3  //0.6 for outdoor //FIXME
		// 	    if(mbUseIMU && !mpLocalMapper->GetVINSInited())
		// 		timegap = 0.15;


			// Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
			//const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
			const bool c1a = (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp >= largetimegap);
			// Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
			const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
			// Condition 1c: tracking is weak
			const bool c1c = (mnMatchesInliers < 50) || bNeedToInsertClose;

			// Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
			const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

			// in case the drift of bias (IMU aided)
			const bool cTimeGap =
				/*mbUseIMU && */((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= timegap) && bLocalMappingIdle;


				
				
			if (((c1a || c1b || c1c) && c2) || cTimeGap) {
			// If the mapping accepts keyframes, insert keyframe.
			// Otherwise send a signal to interrupt BA
			if (bLocalMappingIdle) {
				cout << c1a <<"|"<<c1b <<"|"<<c1c<<"|"<<c2 <<"|"<<cTimeGap << endl;
				return true;
			} else {
				mpLocalMapper->InterruptBA();
				// if(mSensor!=System::MONOCULAR)   // why != monocular?
				{
					if (mpLocalMapper->KeyframesInQueue() < 3)
						return true;
					else
						return false;
				}
				// else
				// return false;
			}
			} else
			return false;
		}
		else//KF selection using HKUST's strategy
		{
			//First Condition: first two frames
			if(nKFs < 12)
				return true;
			
			
			//////////////// V2.0 /////////////////////////
			// parallax
			ORBmatcher matcher(0.7, true);
			if(matcher.SearchParralax(mCurrentFrame, mpLastKeyFrame) > 	1.0)
			{
		// 	      cout << "Add a init KF" << endl;
				return true;
			}
			return false;
		}
	}


	void Tracking::CreateNewKeyFrame() {
		cout << "create new keyframe" << endl;
		Timer tTrackCNKF;
		if (!mpLocalMapper->SetNotStop(true))
			return;

		// if we have not extracted features, we do feature extraction here
		if (mCurrentFrame.mbFeatureExtracted == false) {
			// this key frame is generated with direct tracking
			mCurrentFrame.ExtractFeatures();
			// update the map point cache ?
		} else {
			// this is generated with feature matching, which may occur in monocular initialization or track with reference keyframe
		}


		KeyFrame *pKF = nullptr;
		if(mpLastKeyFrame->GetInitializedByVIO()){
			SE3f Tlr = mlRelativeFramePoses.back();
			mCurrentFrame.SetPose(Tlr * mpReferenceKF->GetPose());
		}

		pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
	// 		pKF->Tlc = mpLaserMap->Tlc;//TODO
		

		
		mpReferenceKF = pKF;
		mCurrentFrame.mpReferenceKF = pKF;
		pKF->nmatches = mCurrentFrame.nmatches;

		if (mSensor != System::MONOCULAR) {
			// in RGBD and stereo mode, we can generate new feature with only one key-frame
			mCurrentFrame.UpdatePoseMatrices();
			// We sort points by the measured depth by the stereo/RGBD sensor.
			// We create all those MapPoints whose depth < mThDepth.
			// If there are less than 100 close points we create the 100 closest.
			vector<pair<float, int> > vDepthIdx;
			vDepthIdx.reserve(mCurrentFrame.N);
			for (int i = 0; i < mCurrentFrame.N; i++) {
				float z = mCurrentFrame.mvDepth[i];
				if (z > 0) {
					vDepthIdx.push_back(make_pair(z, i));
				}
			}

			if (!vDepthIdx.empty()) {
				sort(vDepthIdx.begin(), vDepthIdx.end());

				int nPoints = 0;
				for (size_t j = 0; j < vDepthIdx.size(); j++) {
					int i = vDepthIdx[j].second;

					bool bCreateNew = false;

					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

					if (!pMP)
						bCreateNew = true;
					else if (pMP->Observations() < 1) {
						bCreateNew = true;
						mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
					}

					if (bCreateNew) {
						Vector3f x3D = mCurrentFrame.UnprojectStereo(i);
						MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
						pNewMP->AddObservation(pKF, i);
						pKF->AddMapPoint(pNewMP, i);
						pNewMP->ComputeDistinctiveDescriptors();
						pNewMP->UpdateNormalAndDepth();
						mpMap->AddMapPoint(pNewMP);
						mvpDirectMapPointsCache.insert(pNewMP);
						mCurrentFrame.mvpMapPoints[i] = pNewMP;
						nPoints++;
					} else {
						nPoints++;
					}

					if (vDepthIdx[j].first > mThDepth && nPoints>100)
						break;
				}
			}
		}
		
		

		mpLocalMapper->InsertKeyFrame(pKF);
		mpLocalMapper->SetNotStop(false);

		mnLastKeyFrameId = mCurrentFrame.mnId;
		mpLastKeyFrame = pKF;

		if (mpMap->GetAllKeyFrames().size() > 10)
			mMinFrames = 10;
		float tTCNKF = tTrackCNKF.elapsed();
		if (mverbose)
			frouteTrack << "CreateNewKeyFrame: " << tTCNKF << " [ms]." << endl;
	}	
	
	
	bool Tracking::printParams(std::ofstream &ofile){
		if (!ofile)
			return false;

		ofile << "mMaxFrames: " << mMaxFrames << endl;
		ofile << "Tracking.CacheFeatures: " << mnCacheHitTh << endl;
		ofile << "minInitFeaturePoints: " << minInitFeaturePoints << endl;
		ofile << "minInitMapPoints: " << minInitMapPoints << endl;
		ofile << "minInitTrackPoints: " << minInitTrackPoints << endl;
		ofile << "minTrackPoints: " << minTrackPoints << endl;
		ofile << "minLocalMapTrackPoints: " << minLocalMapTrackPoints << endl;

		ofile << endl << "ORB Extractor Parameters: " << endl;
		ofile << "- Number of Features: " << nFeatures << endl;
		ofile << "- Scale Levels: " << nLevels << endl;
		ofile << "- Scale Factor: " << fScaleFactor << endl;
		ofile << "- Initial Fast Threshold: " << fIniThFAST << endl;
		ofile << "- Minimum Fast Threshold: " << fMinThFAST << endl;

		if (mSensor == System::STEREO || mSensor == System::RGBD) {
			cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
		}
		if (mSensor == System::RGBD) {
			ofile << "mDepthMapFactor: " << mDepthMapFactor << endl;
		}

		ofile << "mK: " << endl << mK << endl;
		//ofile << "mDistCoef: " << endl << mDistCoef.t() << endl;
		ofile << "mDistCoef: " << endl;
		ofile << "- p1: " << mDistCoef.at<float>(2) << endl;
		ofile << "- p2: " << mDistCoef.at<float>(3) << endl;
		ofile << "- k1: " << mDistCoef.at<float>(0) << endl;
		ofile << "- k2: " << mDistCoef.at<float>(1) << endl;
		if (mDistCoef.rows == 5)
			ofile << "- k3: " << mDistCoef.at<float>(4) << endl;
		if (mDistCoef.rows == 8) {
			ofile << "- k3: " << mDistCoef.at<float>(4) << endl;
			ofile << "- k4: " << mDistCoef.at<float>(5) << endl;
			ofile << "- k5: " << mDistCoef.at<float>(6) << endl;
			ofile << "- k6: " << mDistCoef.at<float>(7) << endl;
		}
		
		if (mbRGB)
			ofile << "- color order: RGB (ignored if grayscale)" << endl;
		else
			ofile << "- color order: BGR (ignored if grayscale)" << endl;


		return true;
	}
	
	bool Tracking::CheckMapUpdated(void){

		bool bMapUpdated = false;
		if (mpLocalMapper->GetMapUpdateFlagForTracking()) {
			bMapUpdated = true;
			mpLocalMapper->SetMapUpdateFlagInTracking(false);
		}

	///TEST syywh
	// 	if (mpLoopClosing->GetMapUpdateFlagForTracking()) {
	// 		LOG(INFO) << "Tracking noted that map is updated by loop closing" << endl;
	// 		bMapUpdated = true;
	// 		mpLoopClosing->SetMapUpdateFlagInTracking(false);
	// 	}

		return bMapUpdated;
	}
	
	void Tracking::SetViewer(Viewer *pViewer) {
		mpViewer = pViewer;
	}
	
	void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
		mpLocalMapper = pLocalMapper;
	}
	
		
	/**
	*@brief update mvpLocalKeyFrames 
	* mvpLocalKeyFrames is cleared, then add\n
	* keyframes observing the points in current frame
	* keyframes in covisibility graph with current mvpLocalKeyFrames
	* keyframes from the children
	* keyframes from the parents
	*/
	void Tracking::UpdateLocalKeyFrames() {
		// Each map point vote for the keyframes in which it has been observed
		map<KeyFrame *, int> keyframeCounter;
		for (int i = 0; i < mCurrentFrame.N; i++) {
			if (mCurrentFrame.mvpMapPoints[i]) {
				MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
				if (!pMP->isBad()) {
					const map<KeyFrame *, size_t> observations = pMP->GetObservations();
					for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end();
							it != itend; it++)
						keyframeCounter[it->first]++;
				} else {
					mCurrentFrame.mvpMapPoints[i] = NULL;
				}
			}
		}

		if (keyframeCounter.empty())
			return;

		int max = 0;
		KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

		mvpLocalKeyFrames.clear();
		mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

		// All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
		for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
				it != itEnd; it++) {
			KeyFrame *pKF = it->first;

			if (pKF->isBad())
				continue;

			if (it->second > max) {
				max = it->second;
				pKFmax = pKF;
			}

			mvpLocalKeyFrames.push_back(it->first);
			pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
		}


		// Include also some not-already-included keyframes that are neighbors to already-included keyframes
		for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
				itKF != itEndKF; itKF++) {
			// Limit the number of keyframes
			if (mvpLocalKeyFrames.size() > 80)
				break;

			KeyFrame *pKF = *itKF;

			const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

			for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
					itNeighKF != itEndNeighKF; itNeighKF++) {
				KeyFrame *pNeighKF = *itNeighKF;
				if (!pNeighKF->isBad()) {
					if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
						mvpLocalKeyFrames.push_back(pNeighKF);
						pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
						break;
					}
				}
			}

			const set<KeyFrame *> spChilds = pKF->GetChilds();
			for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
				KeyFrame *pChildKF = *sit;
				if (!pChildKF->isBad()) {
					if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
						mvpLocalKeyFrames.push_back(pChildKF);
						pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
						break;
					}
				}
			}

			KeyFrame *pParent = pKF->GetParent();
			if (pParent) {
				if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
					mvpLocalKeyFrames.push_back(pParent);
					pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
					break;
				}
			}
		}

		if (pKFmax) {
			mpReferenceKF = pKFmax;
			mCurrentFrame.mpReferenceKF = mpReferenceKF;
		}
	}


	/**
	* @brief Update Local MapPoints，called by UpdateLocalMap()
	*
	* 1. traverse mvpLocalKeyFrames, update mnTrackReferenceForFrame for their related map points, update mvpLocalMapPoints
	* 2. update mvpLocalMapPoints by the map points observed in last Keyframe
	*/
	void Tracking::UpdateLocalPoints() {
		mvpLocalMapPoints.clear();

		for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
				itKF != itEndKF; itKF++) {
			KeyFrame *pKF = *itKF;
			const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

			for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
					itMP != itEndMP; itMP++) {
				MapPoint *pMP = *itMP;
				if (!pMP)
					continue;
				if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
					continue;
				if (!pMP->isBad()) {
					mvpLocalMapPoints.push_back(pMP);
					pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
				}
			}
		}

		for (MapPoint *mp: mpLastKeyFrame->GetMapPointMatches()) {
			if (mp == nullptr || mp->isBad() || mp->mnTrackReferenceForFrame == mCurrentFrame.mnId)
				continue;
			mvpLocalMapPoints.push_back(mp);
		}
	}

	
	

	/**
	* @brief: Searching points in Local map which are in the FOV of current frame
	* 		   Project those points into current frame then matching them with ExtractFeatures
	* 		   To expand the associated map points, before which only points in last KF or Frame be matched
	*/
	void Tracking::SearchLocalPoints() {
		// Do not search map points already matched
		for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end();
			vit != vend; vit++) {
			MapPoint *pMP = *vit;
			if (pMP) {
				if (pMP->isBad()) {
					*vit = static_cast<MapPoint *>(NULL);
				}
				else {
					pMP->IncreaseVisible();
					pMP->mnLastFrameSeen = mCurrentFrame.mnId;
					pMP->mbTrackInView = false;
				}
			}
		}

		int nToMatch = 0;

		// Project points in frame and check its visibility
		for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
			vit != vend; vit++) {
			MapPoint *pMP = *vit;
			if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
				continue;
			if (pMP->isBad())
				continue;
			// Project (this fills MapPoint variables for matching)
			if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
				pMP->IncreaseVisible();
				nToMatch++;
			}
		}

		if (nToMatch > 0) {
			ORBmatcher matcher(0.8);
			bool checkLevel = true;
			int th = 1;
			if (mSensor == System::RGBD)
				th = 3;
			// If the camera has been relocalised recently, perform a coarser search
			if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
				th = 5;
			if (mbDirectFailed) { // direct method wouldn't be used here
				LOG(ERROR) << "Direct Failed, should not exist here" << endl;
				checkLevel = false;
				th = 5;
			}
			int cnt = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, false);
			
	// 			if(cnt < 20)
	// 			{
	// 			    cout << "low confidence, try increasing threashold!" << endl;
	// 			    th = 20;
	// 			    cnt = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, false);
	// 			}
			
			frouteTrack << "SearchLocalPoints: " << cnt << endl;
		}
	}
	

		
	/**
	* @brief Update LocalMap
	*
	* Local Map includes: \n
	* - K1 keyframes,K2 neighbor KeyFrame, ref Keyframe
	* - observed MapPoints
	*/
	void Tracking::UpdateLocalMap() {
		// This is for visualization
		mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
		// Update
		UpdateLocalKeyFrames();
		UpdateLocalPoints();
	}
	
		
	bool Tracking::Relocalization_PnP() {
		Timer tReloc;
		// in relocalization we need to extract the features, direct method cannot help us
		if (mCurrentFrame.mbFeatureExtracted == false) {
			mCurrentFrame.ExtractFeatures();
		}
	// 	cout << "Relocalization for mCurrentFrame=" << mCurrentFrame.mnId << endl;

		// Compute Bag of Words Vector
		// mCurrentFrame.ComputeBoW();

		// Relocalization is performed when tracking is lost
		// Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
		vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

		if (vpCandidateKFs.empty()) {
			return false;
		}

		const int nKFs = vpCandidateKFs.size();

		// We perform first an ORB matching with each candidate
		// If enough matches are found we setup a PnP solver
		ORBmatcher matcher(0.75, true);
		// ORBmatcher matcher(0.9,false);

		vector<PnPsolver *> vpPnPsolvers;
		vpPnPsolvers.resize(nKFs);

		vector<vector<MapPoint *> > vvpMapPointMatches;
		vvpMapPointMatches.resize(nKFs);

		vector<bool> vbDiscarded;
		vbDiscarded.resize(nKFs);

		int nCandidates = 0;

		vector<int> candidate_for_save;
		
		for (int i = 0; i < nKFs; i++) {
			KeyFrame *pKF = vpCandidateKFs[i];
			if (pKF->isBad())
				vbDiscarded[i] = true;
			else {
				int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
				if (nmatches < 10) {
					vbDiscarded[i] = true;
					continue;
				}
				else {
					PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
					pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
					vpPnPsolvers[i] = pSolver;
					nCandidates++;
					candidate_for_save.push_back(i);
					
				}
			}
		}

		// Alternatively perform some iterations of P4P RANSAC
		// Until we found a camera pose supported by enough inliers
		bool bMatch = false;
		ORBmatcher matcher2(0.9, true);
		int maxGood = -1;
		while (nCandidates > 0 && !bMatch) {
	// 		cout <<"relocalization" << endl;
	// 		frtPose<<std::to_string(mCurrentFrame.mTimeStamp)<<" " ;
	// 		for(int i =0; i < candidate_for_save.size();i++)
	// 			frtPose<< std::to_string(vpCandidateKFs[i]->mTimeStamp) <<" ";
	// 		frtPose<<endl;
			lastRelocTime = mCurrentFrame.mTimeStamp;
			
			
			for (int i = 0; i < nKFs; i++) {
				if (vbDiscarded[i])
					continue;

				// Perform 5 Ransac Iterations
				vector<bool> vbInliers;
				int nInliers;
				bool bNoMore;

				PnPsolver *pSolver = vpPnPsolvers[i];
				cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

				// If Ransac reachs max. iterations discard keyframe
				if (bNoMore) {
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If a Camera Pose is computed, optimize
				if (!Tcw.empty()) {
					auto tmp = Converter_g2o::toSE3Quat(Tcw);
					mCurrentFrame.mTcw = SE3d(tmp.rotation(), tmp.translation()).cast<float>();
					mCurrentFrame.mbPoseSet = true;

					set<MapPoint *> sFound;

					const int np = vbInliers.size();

					for (int j = 0; j < np; j++) {
						if (vbInliers[j]) {
							mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
							sFound.insert(vvpMapPointMatches[i][j]);
						}
						else
							mCurrentFrame.mvpMapPoints[j] = NULL;
					}

					int nGood = PoseOpt(&mCurrentFrame);

					if (nGood < 10) {
						continue;
					}

					for (int io = 0; io < mCurrentFrame.N; io++)
						if (mCurrentFrame.mvbOutlier[io])
							mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

					// If few inliers, search by projection in a coarse window and optimize again
					if (nGood < 50) {
						int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10,
							100);

						if (nadditional + nGood >= 50) {
							nGood = PoseOpt(&mCurrentFrame);

							// If many inliers but still not enough, search by projection again in a narrower window
							// the camera has been already optimized with many points
							if (nGood > 30 && nGood < 50) {
								sFound.clear();
								for (int ip = 0; ip < mCurrentFrame.N; ip++)
									if (mCurrentFrame.mvpMapPoints[ip])
										sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
								nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3,
									64);

								// Final optimization
								if (nGood + nadditional >= 50) {
									nGood = PoseOpt(&mCurrentFrame);

									for (int io = 0; io < mCurrentFrame.N; io++)
										if (mCurrentFrame.mvbOutlier[io])
											mCurrentFrame.mvpMapPoints[io] = NULL;
								}
							}
						}
					}

					if (nGood > maxGood)
						maxGood = nGood;
					// If the pose is supported by enough inliers stop ransacs and continue
					if (nGood >= 50) {
						bMatch = true;
						break;
					}
				}
	// 			else
	// 				cout << "over,waiting for next DBOW...\n";
			}
		}
		double ttReloc = tReloc.elapsed();
		frouteTrack << "Reloc: " << "nKFs: " << nKFs<<" "<< maxGood << " " << ttReloc << " [ms]." << endl;
		if (!bMatch) {
			return false;
		}
		else {
			mnLastRelocFrameId = mCurrentFrame.mnId;
			cout << "-----------------------------------relocalization success\n";
	// 		frtPose <<"relocalization success PnP "<< to_string(mCurrentFrame.mTimeStamp) << endl
	// 		<< mCurrentFrame.mTcw.inverse().matrix()<< endl;
			
			Eigen::Matrix4f mTwc = mCurrentFrame.mTcw.inverse().matrix();
			Eigen::Quaternionf mTwcq(mTwc.block<3,3>(0,0));
			frtPose << to_string(mCurrentFrame.mTimeStamp) <<" " << mTwc(0,3)<<" " << mTwc(1,3)<<" " << mTwc(2,3)<<" "<<
			mTwcq.x() <<" " << mTwcq.y() <<" " << mTwcq.z()<<" " << mTwcq.w() << " 0"<< endl;
			return true;
		}

	}

		
	void Tracking::setRelocalizationState(bool flag){
		unique_lock<mutex> lock(mMutexReLocalizationUpdate);
		mbRelocalizationSuccess = flag;
	}

	bool Tracking::getRelocalizationState(){
		unique_lock<mutex> lock(mMutexReLocalizationUpdate);
		return mbRelocalizationSuccess;
	}
	
		
	void Tracking::setPubLoc(bool flag){
		unique_lock<mutex> lock(mMutexPubLoc);
		mbPubLoc = flag;
	}

	bool Tracking::getPubLoc(){
		unique_lock<mutex> lock(mMutexPubLoc);
		return mbPubLoc;
	}
}