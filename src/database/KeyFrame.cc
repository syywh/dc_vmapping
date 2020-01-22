#include "KeyFrame.h"

#include "KeyFrameDatabase.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "Map.h"

#include <iomanip>
#include <glog/logging.h>


namespace vill {

    long unsigned int KeyFrame::nNextId = 0;

    void KeyFrame::EraseImage()
    {
	unique_lock<mutex> lock(mMutexEraseImage);
	this->mvImagePyramid.clear();
    }
    
    void KeyFrame::setInitKF(bool flag) {
        unique_lock<mutex> lock(mMutexInitKF);
        bIsInitKF = flag;
    }
    
    bool KeyFrame::isInitKF()
    {
	unique_lock<mutex> lock(mMutexInitKF);
	return bIsInitKF;
    }
    
    
    


    KeyFrame *KeyFrame::GetPrevKeyFrame(void) {
        unique_lock<mutex> lock(mMutexPrevKF);
        return mpPrevKeyFrame;
    }

    KeyFrame *KeyFrame::GetNextKeyFrame(void) {
        unique_lock<mutex> lock(mMutexNextKF);
        return mpNextKeyFrame;
    }

    void KeyFrame::SetPrevKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexPrevKF);
        mpPrevKeyFrame = pKF;
    }

    void KeyFrame::SetNextKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexNextKF);
        mpNextKeyFrame = pKF;
    }

 
    // regular constructor
    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
            mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), mpPrevKeyFrame(nullptr),
            mpNextKeyFrame(nullptr),save_updated(0),
            bIsInitKF(false),initializedByVIO(false) {
        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw);
        for (cv::Mat &im: F.mvImagePyramid) {
            mvImagePyramid.push_back(im);
        }

    }
    

	KeyFrame::KeyFrame(Map* pMap, Frame& F):
		mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
		mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
		mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
		mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
		fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
		mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys),
		mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
		mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
		mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
		mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
		mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints),
		mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
		mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
	{

	}


    void KeyFrame::ComputeBoW() {
        if (mBowVec.empty() || mFeatVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void KeyFrame::SetPose(const SE3f &Tcw_) {
        unique_lock<mutex> lock(mMutexPose);
	
    if((save_updated == 1) || (save_updated == 0)){
      Twc_ori = Twc;
      Tcw_ori = Tcw;
      Ow_ori = Ow;
      
//       Matrix4d Twc_ori_matrix4d = LaserMap::RefineKeyFrameOriPoseTwc(Twc.matrix());
//       Twc_ori =SE3f(Twc_ori_matrix4d.cast<float>());
//       Tcw_ori = Twc_ori.inverse();
// //       Twc_ori = Twc;
// //       mTcp_ori = mTcp;
//       
//       Ow_ori = LaserMap::RefineKeyFrameOriPoseOw(Ow).cast<float>();
      
    }
     save_updated ++;
     
        Tcw = Tcw_;
        Matrix3f Rcw = Tcw.rotationMatrix();
        Vector3f tcw = Tcw.translation();
        Matrix3f Rwc = Rcw.transpose();
        Ow = -Rwc * tcw;

        Twc = SE3f(Rwc, Ow);
        Vector3f center(mHalfBaseline, 0, 0);
        Cw = (Twc * center);
    }
    
	void KeyFrame::SetLaserPose(const Matrix4f& Tlw)
	{
		mlaserPose = Tlw;
	}
	
	Matrix4f KeyFrame::GetLaserpose()
	{
		return mlaserPose;
	}
	
	void KeyFrame::SetLaserposeTimestamp(const double& timestamplaser)
	{
		laserPoseTimestamp = timestamplaser; 
	}

	double KeyFrame::GetLaserposeTimestamp()
	{
		return laserPoseTimestamp;
	}



    SE3f KeyFrame::GetPose() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw;
    }
    
    SE3f KeyFrame::GetPoseOri()
    {
	unique_lock<mutex> lock(mMutexPose);
	if((save_updated>1))
	return Tcw_ori;
	else
	  return Tcw;
    }


    SE3f KeyFrame::GetPoseInverse() {
        unique_lock<mutex> lock(mMutexPose);
        return Twc;
    }

SE3f KeyFrame::GetPoseInverseOri()
{
   unique_lock<mutex> lock(mMutexPose);
    if((save_updated>1))
    return Tcw_ori;
    else
      return Tcw;
}

    
    Vector3f KeyFrame::GetCameraCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Ow;
    }

    Vector3f KeyFrame::GetStereoCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Cw;
    }

    Matrix3f KeyFrame::GetRotation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rotationMatrix();
    }

    Vector3f KeyFrame::GetTranslation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.translation();
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight)
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    void KeyFrame::UpdateBestCovisibles() {
        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));

        sort(vPairs.begin(), vPairs.end());
        list < KeyFrame * > lKFs;
        list<int> lWs;
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin();
             mit != mConnectedKeyFrameWeights.end(); mit++)
            s.insert(mit->first);
        return s;
    }

    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
        unique_lock<mutex> lock(mMutexConnections);
        if ((int) mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

    }

    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
        unique_lock<mutex> lock(mMutexConnections);

        if (mvpOrderedConnectedKeyFrames.empty())
            return vector<KeyFrame *>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                               KeyFrame::weightComp);
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = pMP;
    }


    
    void KeyFrame::EraseMapPointMatch(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx >= 0)
            mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }


    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) {
        mvpMapPoints[idx] = pMP;
    }

    set<MapPoint *> KeyFrame::GetMapPoints() {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int KeyFrame::TrackedMapPoints(const int &minObs) {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        for (int i = 0; i < N; i++) {
            MapPoint *pMP = mvpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (bCheckObs) {
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    } else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    vector<MapPoint *> KeyFrame::GetMapPointMatches() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx];
    }

    void KeyFrame::UpdateConnections() {
        map<KeyFrame *, int> KFcounter;

        vector<MapPoint *> vpMP;

        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

            map<KeyFrame *, size_t> observations = pMP->GetObservations();

            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                if (mit->first->mnId == mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        // 共视超过15个点就被认为是有connection了。。。然而直接法中共视超多的
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;
        // int th = 30;

        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
            if (mit->second > nmax) {
                nmax = mit->second;
                pKFmax = mit->first;
            }
            if (mit->second >= th) {
                vPairs.push_back(make_pair(mit->second, mit->first));
                (mit->first)->AddConnection(this, mit->second);
            }
        }

        if (vPairs.empty()) {
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }

        sort(vPairs.begin(), vPairs.end());
        list < KeyFrame * > lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if (mbFirstConnection && mnId != 0) {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }

        }
//         cout << "covisibility graph size " << mvpOrderedConnectedKeyFrames.size() << endl;
    }

    void KeyFrame::AddChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame *KeyFrame::GetParent() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }
    
	void KeyFrame::SetInitializedByVIO()
	{
		unique_lock<mutex> lockCon(mMutexInitVIO);
		initializedByVIO = true;

	}
	
	void KeyFrame::ResetInitializedByVIO()
	{
		unique_lock<mutex> lockCon(mMutexInitVIO);
		initializedByVIO = false;

	}

	bool KeyFrame::GetInitializedByVIO()
	{
		unique_lock<mutex> lockCon(mMutexInitVIO);
		return initializedByVIO;

	}


    void KeyFrame::SetNotErase() {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty()) {
                mbNotErase = false;
            }
        }

        if (mbToBeErased) {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag() {
        // Test log
        if (mbBad) {
            vector<KeyFrame *> vKFinMap = mpMap->GetAllKeyFrames();
            std::set<KeyFrame *> KFinMap(vKFinMap.begin(), vKFinMap.end());
            if (KFinMap.count(this)) {
                mpMap->EraseKeyFrame(this);
            }
            mpKeyFrameDB->erase(this);
            return;
        }

        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mnId == 0) {
                LOG(INFO) << "Don't erase the first key frame" << endl;
                return;
            } else if (mbNotErase) {
                mbToBeErased = true;
                return;
            }
        }

        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            mit->first->EraseConnection(this);

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
            if (mvpMapPoints[i])
                mvpMapPoints[i]->EraseObservation(this);
        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while (!mspChildrens.empty()) {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC=NULL;
                KeyFrame *pP=NULL;

                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end();
                     sit != send; sit++) {
                    KeyFrame *pKF = *sit;
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                             spcit != spcend; spcit++) {
                            if (vpConnected[i]->mnId == (*spcit)->mnId) {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if (w > max) {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if (bContinue) {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                } else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if (!mspChildrens.empty())
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                    (*sit)->ChangeParent(mpParent);
                }

            mpParent->EraseChild(this);
            mTcp = Tcw * mpParent->GetPoseInverse();

            // Update Prev/Next KeyFrame for prev/next
            KeyFrame *pPrevKF = GetPrevKeyFrame();
            KeyFrame *pNextKF = GetNextKeyFrame();
            if (pPrevKF)
                pPrevKF->SetNextKeyFrame(pNextKF);
            if (pNextKF)
                pNextKF->SetPrevKeyFrame(pPrevKF);
            SetPrevKeyFrame(NULL);
            SetNextKeyFrame(NULL);
//             // TODO
//             if (pPrevKF && pNextKF) {
//                 // Update IMUData for NextKF
//                 pNextKF->AppendIMUDataToFront(this);
//                 // Re-compute pre-integrator
//                 pNextKF->ComputePreInt();
//             }

            mbBad = true;
        }

// 	this->mvImagePyramid.clear();
        
        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool KeyFrame::isBad() {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF) {
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF)) {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

        if (bUpdate)
            UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int) mnGridCols - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int) mnGridRows - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeys[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    Vector3f KeyFrame::UnprojectStereo(int i) {
        const float z = mvDepth[i];
        if (z > 0) {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Vector3f x3Dc(x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc * x3Dc;
        } else
            return Vector3f(0, 0, 0);
    }

    float KeyFrame::ComputeSceneMedianDepth(const int q) {
        vector<MapPoint *> vpMapPoints;
        SE3f Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            Tcw_ = Tcw;
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        Vector4f r = Tcw_.matrix().row(2);
        Vector3f Rcw2 = r.head<3>();
        float zcw = Tcw_.matrix()(2, 3);

        for (int i = 0; i < N; i++) {
            if (mvpMapPoints[i]) {
                MapPoint *pMP = mvpMapPoints[i];
                Vector3f x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw;
                vDepths.push_back(z);
            }
        }
        sort(vDepths.begin(), vDepths.end());

        return vDepths[(vDepths.size() - 1) / q];
    }
    
	map< int, int > KeyFrame::getConnectedKeyFrameWeights()
	{
		mConnectedKF_weights.clear();

		for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++){
			if(mit->first)
				mConnectedKF_weights[mit->first->mnId] = mit->second;
		}
		return mConnectedKF_weights;

	}

} //namespace vill
