#ifndef OPTIMIZER_G2O_H_
#define OPTIMIZER_G2O_H_


// #include "LoopClosing.h"

#include "thirdparty/sophus/sophus/se3.hpp"



#include "g2otypes.h"
#include <Eigen/Core>
using namespace Eigen;


namespace vill {

//     class LoopClosing;
	class LocalMapping;
	
	class Map;
	class Frame;
	class KeyFrame;
	class MapPoint;

    class Optimizer_g2o {
    private:
	static MatrixXd TangentBasis(Vector3d &g0);
	bool static LinearAlignment(list<KeyFrame*> &all_image_frame, Vector3d &g, VectorXd &x);
	void static RefineGravity(list<KeyFrame*> &all_image_frame, Vector3d &g, VectorXd &x);
	
      
    public:

	static float distTH;
	static float invariatDist;
	static int searchN;
	static int iteration_count;
	static float p2pInfo;
	static float p2planeInfo;
	static float relativeInfo;
	static float relativeReproInfo;
	static float prioriInfo;
	static float delta;
	static float ceres_iteration_times;
	static float ceres_th_reprojection;
	static float ceres_th_laser;
	static float ceres_th_preintegration;
	static float ceres_th_positionUp;
	static float ceres_th_velocityUp;
	static float ceres_th_rotationUp;
	static float ceres_th_biasaUp;
	static float ceres_th_biasgUp;
	static float ceres_th_prioriUp;
	static float th_iter_icp_fail;
    public:
        // vision only, g2o solver
        static void  BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
                                     int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
		
	

        static void  GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                           const unsigned long nLoopKF = 0, const bool bRobust = true);

        static void  LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, LocalMapping *pLM = NULL);
		

        static int  PoseOptimization(Frame *pFrame);
		

//         // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
//         void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
//                                            const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
//                                            const LoopClosing::KeyFrameAndPose &CorrectedSim3,
//                                            const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
//                                            const bool &bFixScale, LoopClosing *pLC = NULL);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
        static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
                                g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
	
		static inline double norm(Vector3d r){
		return ( r[0]*r[0] + r[1] * r[1] + r[2] * r[2]);
		}
	
		static fstream f;
	
    };

} //namespace vill

#endif // OPTIMIZER_H

