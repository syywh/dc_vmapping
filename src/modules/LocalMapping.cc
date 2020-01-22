#include "LocalMapping.h"
#include "LocalMapping.h"
#include "Timer.h"
#include "Map.h"
#include "MapPoint.h"

#include "glog/logging.h"

namespace vill {
	
	Eigen::MatrixXd LocalMapping::info = Eigen::Matrix<double,120,120>::Identity();
		
	LocalMapping::LocalMapping(Map *pMap, const float bMonocular, ConfigParam *pParams) :
		mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
		mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),bMessageUpdate(false){
		mpParams = pParams;
		mnLocalWindowSize = mpParams->GetLocalWindowSize();
		mnInitWindowSize = mpParams->GetInitWindowSize();
		mbUseIMU = mpParams->GetUseIMUFlag();
		mbUseLaserMap = mpParams->GetUseLaserMapFlag();
		mbUseLaserpose = mpParams->GetUseLaserPoseFlag();
		LOG(INFO) << "mnLocalWindowSize:" << mnLocalWindowSize << ", mbUseIMU:" << mbUseIMU << endl;
		
		mverbose = ConfigParam::_verbose;

		mbVINSInited = false;
		mbFirstTry = true;
		mbFirstVINSInited = false;
		mbVINSIniting = false;
		mbResetVINSInit = false;

		mbUpdatingInitPoses = false;
		mbCopyInitKFs = false;

		//Thread for VINS initialization
// 		if (mbUseIMU)
// 		mptLocalMappingVIOInit = new thread(&LocalMapping::VINSInitThread, this);
// 		else
		mptLocalMappingVIOInit = NULL;
		
		string tmpfilepath = ConfigParam::getTmpFilePath();
		output_gw.open(tmpfilepath + "gravity.txt");
		output_biasa.open(tmpfilepath + "biasa.txt");
		output_biasg.open(tmpfilepath + "biasg.txt");
		output_v.open(tmpfilepath + "velocity.txt");
		
		localizationState = 1;//OK&&DONE
		info.resize(6*mnLocalWindowSize, 6*mnLocalWindowSize);
		bmappoints_updated_by_localization = false;
		blocalkeyframes_updated_by_localization = false;
		

		
	}
	
	void LocalMapping::ProcessNewKeyFrame() {
		{
			unique_lock<mutex> lock(mMutexNewKFs);
			// retrieve first KF need to be inserted
			mpCurrentKeyFrame = mlNewKeyFrames.front();
			mlNewKeyFrames.pop_front();
		}
		
		if (mverbose)
		cout << "KF: " << mpCurrentKeyFrame->mnId << " " << mpCurrentKeyFrame->mnFrameId << endl;

		Timer tLM;
		// Associate MapPoints to the new keyframe and update normal and descriptor
		const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

		for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
			MapPoint *pMP = vpMapPointMatches[i];
			if (pMP) {
				if (!pMP->isBad()) {
					// NOTE
					if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
						pMP->AddObservation(mpCurrentKeyFrame, i);
						pMP->UpdateNormalAndDepth();
						pMP->ComputeDistinctiveDescriptors();
					} else // this can only happen for new stereo points inserted by the Tracking
					{
						mlpRecentAddedMapPoints.push_back(pMP);
					}
				}
			}
		}

		// Update links in the Covisibility Graph
		mpCurrentKeyFrame->UpdateConnections();

		// Delete bad KF in LocalWindow
		DeleteBadInLocalWindow();
		// Add Keyframe to LocalWindow
		AddToLocalWindow(mpCurrentKeyFrame);

		// Insert Keyframe in Map
		mpMap->AddKeyFrame(mpCurrentKeyFrame);
			
		float tPNKF = tLM.elapsed();
		if (mverbose)
		cout << "ProcessNewKeyFrame: " << tPNKF << " ms." << endl;
	}
	
	void LocalMapping::Run() {
		
		Timer tLM;

		mbFinished = false;
		
		while (1) {
			// Tracking will see that Local Mapping is busy
			SetAcceptKeyFrames(false);
			
			// Check if there are keyframes in the queue
			if (CheckNewKeyFrames()) {
				// BoW conversion and insertion in Map
				ProcessNewKeyFrame();
				
				MapPointCulling();
				
				CreateNewMapPoints();
				
				if (!CheckNewKeyFrames()) {
					// Find more matches in neighbor keyframes and fuse point duplications
					SearchInNeighbors();
				}
				
				mbAbortBA = false;
				
				if (!CheckNewKeyFrames() && !stopRequested()) {
					// VI-D Local BA
					tLM.reset();
					if (mpMap->KeyFramesInMap() > 2) {
						LocalBA();
					}

					KeyFrameCulling();
				}
				
				
				
			} else if (Stop()){
				while (isStopped() && !CheckFinish()) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
				if (CheckFinish())
				break;
			}
			
			SetAcceptKeyFrames(true);
			
			if (CheckFinish())
				break;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		SetFinish();
	}
	
	void LocalMapping::RunOffline()
	{
		Timer tLM;

		mbFinished = false;
		
		// Tracking will see that Local Mapping is busy
		SetAcceptKeyFrames(false);
		
		if (CheckNewKeyFrames()) {
			// BoW conversion and insertion in Map
				ProcessNewKeyFrame();
				
				MapPointCulling();
				
				CreateNewMapPoints();
				
				if (!CheckNewKeyFrames()) {
					// Find more matches in neighbor keyframes and fuse point duplications
					SearchInNeighbors();
				}
				
				mbAbortBA = false;
				
				if (!CheckNewKeyFrames() && !stopRequested()) {
					// VI-D Local BA
					tLM.reset();
					if (mpMap->KeyFramesInMap() > 2) {
						LocalBA();
					}

					KeyFrameCulling();
				}
		}
		
		SetAcceptKeyFrames(true);
		
		LOG(INFO) <<"Time spent in localMapping "<<tLM.elapsed() << " ms.";

	}

	
}