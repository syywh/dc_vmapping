#include "LocalMapping.h"
#include "MapPoint.h"
#include "Map.h"
#include "Timer.h"
#include "ORBmatcher.h"

#include "glog/logging.h"

namespace vill {
	
	bool LocalMapping::GetUpdatingInitPoses(void) {
		unique_lock<mutex> lock(mMutexUpdatingInitPoses);
		return mbUpdatingInitPoses;
	}
	
	bool LocalMapping::isStopped() {
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}
	
	
	bool LocalMapping::stopRequested() {
		unique_lock<mutex> lock(mMutexStop);
		return mbStopRequested;
	}
	
	void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
		unique_lock<mutex> lock(mMutexNewKFs);
		
		mlNewKeyFrames.push_back(pKF);
		mbAbortBA = true;
	}
	
	bool LocalMapping::SetNotStop(bool flag) {
		unique_lock<mutex> lock(mMutexStop);

		if (flag && mbStopped)
		return false;

		mbNotStop = flag;

		return true;
	}
	
	
	
	void LocalMapping::DeleteBadInLocalWindow(void) {
		std::list<KeyFrame *>::iterator lit = mlLocalKeyFrames.begin();
		while (lit != mlLocalKeyFrames.end()) {
			KeyFrame *pKF = *lit;
			if (!pKF) LOG(ERROR) << "pKF null in DeleteBadInLocalWindow?" << endl; //Test log
			if (pKF->isBad()) {
				lit = mlLocalKeyFrames.erase(lit);
				LOG(INFO) <<"Delete Bad KeyFrame";
			}
			else {
				lit++;
			}
		}
	}
	
	void LocalMapping::AddToLocalWindow(KeyFrame *pKF) {
		mlLocalKeyFrames.push_back(pKF);
		if (mlLocalKeyFrames.size() > mnLocalWindowSize) {
			mlLocalKeyFrames.pop_front();
		}
	}

	void LocalMapping::MapPointCulling() {
		Timer tLM;
		
		// Check Recent Added MapPoints
		list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
		const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

		int nThObs;
		if (mbMonocular)
		nThObs = 2;
		else
		nThObs = 3;
		const int cnThObs = nThObs;

		// check recent added map points
		while (lit != mlpRecentAddedMapPoints.end()) {
		MapPoint *pMP = *lit;
		if (pMP->isBad()) {
			// erase bad points directly
			lit = mlpRecentAddedMapPoints.erase(lit);
		} else if (pMP->GetFoundRatio() < 0.25f) {
			// VI-B cond.1，the tracking must find the point in more than the 25% of the frames in which it is predicted to be visible.
			// IncreaseFound, IncreaseVisible
			pMP->SetBadFlag();
			lit = mlpRecentAddedMapPoints.erase(lit);
		} else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs) {
			// VI-B cond.2，if more than 2 keyframes has passed from map point creation and the number of keyframes which observe this point is less than 2.
			// this point should be set bad
			pMP->SetBadFlag();
			lit = mlpRecentAddedMapPoints.erase(lit);
		} else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 3)
			// the point must be observed from at least 3 keyframes.
			lit = mlpRecentAddedMapPoints.erase(lit);
		else
			lit++;
		}
		float tMPC = tLM.elapsed();
		if (mverbose)
		cout << "MapPointCulling: " << tMPC << " ms." << endl;
	}	
	
	void LocalMapping::CreateNewMapPoints() {
		// Retrieve neighbor keyframes in covisibility graph
		Timer tLM;

		int nn = 10;
		if (mbMonocular)
		nn = 20;
		const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

		ORBmatcher matcher(0.6, false);

		Matrix3f Rcw1 = mpCurrentKeyFrame->GetRotation();
		Matrix3f Rwc1 = Rcw1.transpose();
		Vector3f tcw1 = mpCurrentKeyFrame->GetTranslation();
		Eigen::Matrix<float, 3, 4> Tcw1;
		Tcw1.block<3, 3>(0, 0) = Rcw1;
		Tcw1.block<3, 1>(0, 3) = tcw1;
		Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

		const float &fx1 = mpCurrentKeyFrame->fx;
		const float &fy1 = mpCurrentKeyFrame->fy;
		const float &cx1 = mpCurrentKeyFrame->cx;
		const float &cy1 = mpCurrentKeyFrame->cy;
		const float &invfx1 = mpCurrentKeyFrame->invfx;
		const float &invfy1 = mpCurrentKeyFrame->invfy;

		const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

		int nnew = 0;

		// Search matches with epipolar restriction and triangulate
		for (size_t i = 0; i < vpNeighKFs.size(); i++) {
		if (i > 0 && CheckNewKeyFrames())
			return;

		KeyFrame *pKF2 = vpNeighKFs[i];

		// Check first that baseline is not too short
		Vector3f Ow2 = pKF2->GetCameraCenter();
		Vector3f vBaseline = Ow2 - Ow1;
		const float baseline = vBaseline.norm();

		if (!mbMonocular) {
			if (baseline < pKF2->mb)
			continue;
		} else {
			const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
			const float ratioBaselineDepth = baseline / medianDepthKF2;
			if (ratioBaselineDepth < 0.01)
			continue;
		}

		// Compute Fundamental Matrix
		Matrix3f F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

		// Search matches that fullfil epipolar constraint
		vector<pair<size_t, size_t> > vMatchedIndices;
		matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);

		Matrix3f Rcw2 = pKF2->GetRotation();
		Matrix3f Rwc2 = Rcw2.transpose();
		Vector3f tcw2 = pKF2->GetTranslation();
		Matrix<float, 3, 4> Tcw2;
		Tcw2.block<3, 3>(0, 0) = Rcw2;
		Tcw2.block<3, 1>(0, 3) = tcw2;

		const float &fx2 = pKF2->fx;
		const float &fy2 = pKF2->fy;
		const float &cx2 = pKF2->cx;
		const float &cy2 = pKF2->cy;
		const float &invfx2 = pKF2->invfx;
		const float &invfy2 = pKF2->invfy;

		// Triangulate each match
		const int nmatches = vMatchedIndices.size();
		for (int ikp = 0; ikp < nmatches; ikp++) {
			const int &idx1 = vMatchedIndices[ikp].first;

			const int &idx2 = vMatchedIndices[ikp].second;

			const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeys[idx1];
			const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
			bool bStereo1 = kp1_ur >= 0;

			const cv::KeyPoint &kp2 = pKF2->mvKeys[idx2];
			const float kp2_ur = pKF2->mvuRight[idx2];
			bool bStereo2 = kp2_ur >= 0;

			// Check parallax between rays
			Vector3f xn1((kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
			Vector3f xn2((kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

			Vector3f ray1 = Rwc1 * xn1;
			Vector3f ray2 = Rwc2 * xn2;
			const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

			float cosParallaxStereo = cosParallaxRays + 1;
			float cosParallaxStereo1 = cosParallaxStereo;
			float cosParallaxStereo2 = cosParallaxStereo;

			if (bStereo1)
			cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
			else if (bStereo2)
			cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

			cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

			Vector3f x3D;
			if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
			(bStereo1 || bStereo2 || cosParallaxRays < 0.9998)) {
			// Linear Triangulation Method
			Matrix4f A;
			A.row(0) = xn1[0] * Tcw1.row(2) - Tcw1.row(0);
			A.row(1) = xn1[1] * Tcw1.row(2) - Tcw1.row(1);
			A.row(2) = xn2[0] * Tcw2.row(2) - Tcw2.row(0);
			A.row(3) = xn2[1] * Tcw2.row(2) - Tcw2.row(1);

			Eigen::JacobiSVD<Matrix4f> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Matrix4f V = SVD.matrixV();
			if (V(3, 3) == 0)
				continue;
			x3D = V.block<3, 1>(0, 3) / V(3, 3);

			} else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
			x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
			} else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
			x3D = pKF2->UnprojectStereo(idx2);
			} else
			continue; //No stereo and very low parallax

			Vector3f x3Dt = x3D;

			//Check triangulation in front of cameras
			float z1 = Rcw1.row(2).dot(x3Dt) + tcw1[2];
			if (z1 <= 0)
			continue;

			float z2 = Rcw2.row(2).dot(x3Dt) + tcw2[2];
			if (z2 <= 0)
			continue;

			//Check reprojection error in first keyframe
			const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
			const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1[0];
			const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1[1];
			const float invz1 = 1.0 / z1;

			if (!bStereo1) {
			float u1 = fx1 * x1 * invz1 + cx1;
			float v1 = fy1 * y1 * invz1 + cy1;
			float errX1 = u1 - kp1.pt.x;
			float errY1 = v1 - kp1.pt.y;
			if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
				continue;
			} else {
			float u1 = fx1 * x1 * invz1 + cx1;
			float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
			float v1 = fy1 * y1 * invz1 + cy1;
			float errX1 = u1 - kp1.pt.x;
			float errY1 = v1 - kp1.pt.y;
			float errX1_r = u1_r - kp1_ur;
			if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
				continue;
			}

			//Check reprojection error in second keyframe
			const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
			const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2[0];
			const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2[1];
			const float invz2 = 1.0 / z2;
			if (!bStereo2) {
			float u2 = fx2 * x2 * invz2 + cx2;
			float v2 = fy2 * y2 * invz2 + cy2;
			float errX2 = u2 - kp2.pt.x;
			float errY2 = v2 - kp2.pt.y;
			if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
				continue;
			} else {
			float u2 = fx2 * x2 * invz2 + cx2;
			float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
			float v2 = fy2 * y2 * invz2 + cy2;
			float errX2 = u2 - kp2.pt.x;
			float errY2 = v2 - kp2.pt.y;
			float errX2_r = u2_r - kp2_ur;
			if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
				continue;
			}

			//Check scale consistency
			Vector3f normal1 = x3D - Ow1;
			float dist1 = normal1.norm();

			Vector3f normal2 = x3D - Ow2;
			float dist2 = normal2.norm();

			if (dist1 == 0 || dist2 == 0)
			continue;

			const float ratioDist = dist2 / dist1;
			const float ratioOctave =
				mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

			/*if(fabs(ratioDist-ratioOctave)>ratioFactor)
			continue;*/
			if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
			continue;

			// Triangulation is succesfull
			MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);

			pMP->AddObservation(mpCurrentKeyFrame, idx1);
			pMP->AddObservation(pKF2, idx2);

			mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
			pKF2->AddMapPoint(pMP, idx2);

			pMP->ComputeDistinctiveDescriptors();

			pMP->UpdateNormalAndDepth();

			mpMap->AddMapPoint(pMP);

			mlpRecentAddedMapPoints.push_back(pMP);

			nnew++;
		}
		}
	//     LOG(INFO) << "There are " << nnew << " mappoints created in this step." << endl;
			float tCNMP = tLM.elapsed();
	// 	    if (mverbose)
	// 		    frouteLM << "CreateNewMapPoint: " << tCNMP << endl;
	}

	
	// used in create new map points
	Matrix3f LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2) {
		Matrix3f R1w = pKF1->GetRotation();
		Vector3f t1w = pKF1->GetTranslation();
		Matrix3f R2w = pKF2->GetRotation();
		Vector3f t2w = pKF2->GetTranslation();

		Matrix3f R12 = R1w * R2w.transpose();
		Vector3f t12 = -1 * R1w * R2w.transpose() * t2w + t1w;

		Matrix3f t12x = SO3f::hat(t12);

		const Matrix3f &K1 = pKF1->mK;
		const Matrix3f &K2 = pKF2->mK;

		return K1.transpose().inverse() * t12x * R12 * K2.inverse();
	}
	
	void LocalMapping::KeyFrameCulling() {
		// Check redundant keyframes (only local keyframes)
		// A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
		// in at least other 3 keyframes (in the same or finer scale)
		// We only consider close stereo points
		Timer tLM;
		if (GetFlagCopyInitKFs())
		return;
		
		SetFlagCopyInitKFs(true);
	
		vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

		for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end();
		vit != vend; vit++) {
			KeyFrame *pKF = *vit;
			if (pKF->mnId == 0)
				continue;
			
			// TODO check this
			if ((mpCurrentKeyFrame->mnId - pKF->mnId) <= 10)
				continue;  // don't remove nearby key-frames in vio

			if (mbUseIMU) {
				// Don't drop the KF before current KF
				if (pKF->GetNextKeyFrame() == mpCurrentKeyFrame)
				continue;

				if (pKF->mTimeStamp >= mpCurrentKeyFrame->mTimeStamp - 0.15)
				continue;
			}

			const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

			int nObs = 3;
			const int thObs = nObs;
			int nRedundantObservations = 0;
			int nMPs = 0;
			for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
				MapPoint *pMP = vpMapPoints[i];
				if (pMP) {
					if (!pMP->isBad()) {
						if (!mbMonocular) {
							if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
								continue;
						}

						nMPs++;
						if (pMP->Observations() > thObs) {
							const int &scaleLevel = pKF->mvKeys[i].octave;
							const map<KeyFrame *, size_t> observations = pMP->GetObservations();
							int nObs = 0;
							for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end();
								mit != mend; mit++) {
								KeyFrame *pKFi = mit->first;
								if (pKFi == pKF)
								continue;
								const int &scaleLeveli = pKFi->mvKeys[mit->second].octave;

								if (scaleLeveli <= scaleLevel + 1) {
									nObs++;
									if (nObs >= thObs)
										break;
								}
							}
							if (nObs >= thObs) {
								nRedundantObservations++;
							}
						}
					}
				}
			}

			if (nRedundantObservations > 0.9 * nMPs && (!mbUseIMU )) {
				pKF->SetBadFlag();
			}
		
		}

		SetFlagCopyInitKFs(false);
		float tLMKFC= tLM.elapsed();
		if (mverbose)
		cout << "KeyFrameCulling: " <<  tLMKFC << " ms." <<  endl;
	}
	
	
	void LocalMapping::SearchInNeighbors() {
		Timer tLM;

		// Retrieve neighbor keyframes
		int nn = 10;
		if (mbMonocular)
		nn = 20;
		const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

		vector<KeyFrame *> vpTargetKFs;
		for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++) {
		KeyFrame *pKFi = *vit;
		if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
			continue;
		vpTargetKFs.push_back(pKFi);
		pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

		// Extend to some second neighbors
		const vector<KeyFrame *> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
		for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
			vit2 != vend2; vit2++) {
			KeyFrame *pKFi2 = *vit2;
			if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
			pKFi2->mnId == mpCurrentKeyFrame->mnId)
			continue;
			vpTargetKFs.push_back(pKFi2);
		}
		}


		// Search matches by projection from current KF in target KFs
		ORBmatcher matcher;
		vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
		for (vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++) {
		KeyFrame *pKFi = *vit;

		matcher.Fuse(pKFi, vpMapPointMatches);
		}

		// Search matches by projection from target KFs in current KF
		vector<MapPoint *> vpFuseCandidates;
		vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

		for (vector<KeyFrame *>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end();
		vitKF != vendKF; vitKF++) {
		KeyFrame *pKFi = *vitKF;

		vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

		for (vector<MapPoint *>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end();
			vitMP != vendMP; vitMP++) {
			MapPoint *pMP = *vitMP;
			if (!pMP)
			continue;

			if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
			continue;

			pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
			vpFuseCandidates.push_back(pMP);
		}
		}

		matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);

		// Update points
		vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
		for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++) {
		MapPoint *pMP = vpMapPointMatches[i];
		if (pMP) {
			if (!pMP->isBad()) {
			pMP->ComputeDistinctiveDescriptors();

			pMP->UpdateNormalAndDepth();
			}
		}
		}

		// Update connections in covisibility graph
		mpCurrentKeyFrame->UpdateConnections();
			
		float tSIN = tLM.elapsed();
		if (mverbose)
		cout << "SearchInNeighbors: " << tSIN << " ms." << endl;
	}  
	
	bool LocalMapping::Stop() {
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopRequested && !mbNotStop) {
		mbStopped = true;
		cout << "Local Mapping STOP" << endl;
		return true;
		}

		return false;
	}
	
	
	bool LocalMapping::CheckFinish() {
		unique_lock<mutex> lock(mMutexFinish);
		return mbFinishRequested;
	}
	
	void LocalMapping::SetAcceptKeyFrames(bool flag) {
		unique_lock<mutex> lock(mMutexAccept);
		mbAcceptKeyFrames = flag;
	}
	
	void LocalMapping::SetFinish() {
		unique_lock<mutex> lock(mMutexFinish);
		mbFinished = true;
		unique_lock<mutex> lock2(mMutexStop);
		mbStopped = true;
	}
	
	void LocalMapping::SetMapUpdateFlagInTracking(bool bflag) {
		unique_lock<mutex> lock(mMutexMapUpdateFlag);
		mbMapUpdateFlagForTracking = bflag;
		if (bflag) {
		mpMapUpdateKF = mpCurrentKeyFrame;
		}
	}
	
		
	void LocalMapping::SetTracker(Tracking *pTracker) {
		mpTracker = pTracker;
	}

	void LocalMapping::SetLoopCloser(LoopClosing* pLoopClosure){
		mpLoopCloser = pLoopClosure;
	}
	
	bool LocalMapping::AcceptKeyFrames() {
		unique_lock<mutex> lock(mMutexAccept);
		return mbAcceptKeyFrames;
	}

	
	bool LocalMapping::CheckNewKeyFrames() {
		unique_lock<mutex> lock(mMutexNewKFs);
		return (!mlNewKeyFrames.empty());
	}
	
	bool LocalMapping::GetMapUpdateFlagForTracking() {
		unique_lock<mutex> lock(mMutexMapUpdateFlag);
		return mbMapUpdateFlagForTracking;
	}
	
	bool LocalMapping::isFinished() {
		unique_lock<mutex> lock(mMutexFinish);
		return mbFinished;
	}
	
	void LocalMapping::RequestFinish() {
		unique_lock<mutex> lock(mMutexFinish);
		mbFinishRequested = true;
	}
	
	void LocalMapping::InterruptBA() {
		mbAbortBA = true;
	}

}