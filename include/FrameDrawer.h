#ifndef FRAMEDRAWER_H_
#define FRAMEDRAWER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#include <vector>
#include <mutex>
using namespace std;

namespace vill {

    class Tracking;

    class Viewer;

	class Map;

    class FrameDrawer {
    public:
        FrameDrawer(Map *pMap);
		~FrameDrawer(){
			vw.release();
		}
		
        // Update info from the last processed frame.
        void Update(Tracking *pTracker);

        // Draw last processed frame.
        cv::Mat DrawFrame();
		void Save(void);
		cv::VideoWriter vw;
		static int cnt;
		
    protected:
        void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
		
        // Info of the frame to be drawn
        cv::Mat mIm;
        int N;
        vector<cv::KeyPoint> mvCurrentKeys;
        vector<bool> mvbMap, mvbVO;
        bool mbOnlyTracking;
        int mnTracked, mnTrackedVO;
        vector<cv::KeyPoint> mvIniKeys;
		int mnId;
        vector<int> mvIniMatches;
        int mState;
        vector<int> mvMatchedFrom;

        Map *mpMap;

        std::mutex mMutex;
    };

} //namespace vill

#endif // FRAMEDRAWER_H
