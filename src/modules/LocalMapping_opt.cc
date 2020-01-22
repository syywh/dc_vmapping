#include "LocalMapping.h"
#include "Optimizer_g2o.h"
#include "Timer.h"

namespace vill {
	
	void LocalMapping::LocalBA(void){
		Timer tLM;
		
		LocalBA_onlyVision();
		
		cv::Mat R = Converter::toCvMat(mpCurrentKeyFrame->GetRotation()).t();
		vector<float> q = Converter::toQuaternion(R);
		Vector3f t = mpCurrentKeyFrame->GetCameraCenter();
		
		mPoses[mpCurrentKeyFrame->mTimeStamp] = std::pair<vector<float>, Vector3f>(q, t);
		
		return;

	}
	
	
	void LocalMapping::LocalBA_onlyVision(){

			Optimizer_g2o::LocalBundleAdjustment(
				mpCurrentKeyFrame, &mbAbortBA, mpMap, this);	

	}
}