#include <opencv2/core/core.hpp>
using cv::Mat;

// #include "Thirdparty/g2o/g2o/types/se3quat.h"
// #include "Thirdparty/g2o/g2o/types/sim3.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/sim3/sim3.h"
#include "thirdparty/sophus/sophus/se3.hpp"

// for accelerating the compile

namespace vill {

	class Converter_g2o {
	public:
		static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
		
		static Sophus::SE3d QuattoSE3d(const g2o::SE3Quat& gSim3);
		
		static g2o::SE3Quat toSE3Quat(const Sophus::SE3d &cvT);

		static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

		static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
		
		static Eigen::MatrixXd getBlockMatrix(const Eigen::MatrixXd& originMat, int row, int col, int begin_row, int begin_col);
	};


}
