#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <opencv2/core/core.hpp>
using cv::Mat;
#include <Eigen/Core>
using namespace Eigen;
#include <thirdparty/sophus/sophus/se3.hpp>
#include <thirdparty/sophus/sophus/so3.hpp>
using namespace Sophus;
#include "pointmatcher/PointMatcher.h"

// #include "IMU/NavState.h"

// #include "IMU/IMUPreintegrator.h"

// #include "LaserMap.h"


namespace vill {

    class Converter {
      typedef PointMatcher<float> PM;
      typedef PM::DataPoints DP;
      
    public:
        

        // Convert descriptor from Mat to vector<Mat>
        static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);


		// Convert different data style to Mat
        static cv::Mat toCvMat(const SE3d &SE3);

        static cv::Mat toCvMat(const SE3f &SE3);

        static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);

        static cv::Mat toCvMat(const Eigen::Matrix<float, 4, 4> &m);

        static cv::Mat toCvMat(const Eigen::Matrix3d &m);

        static cv::Mat toCvMat(const Eigen::Matrix3f &m);

        static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);

        static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 1> &m);

        static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t);

        // Convert different data style to Eigen
        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);

        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);

        static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);
		
		static Eigen::Vector3d rotationMatrixToEulerAngles(cv::Mat &R);

		// Convert Mat to quaternion
        static std::vector<float> toQuaternion(const cv::Mat &M);
	
		// Convert Mat to PM::TransformationParameters
		static PM::TransformationParameters toPMTrans(const cv::Mat cvT);
    
    	
    };

}// namespace vill

#endif // CONVERTER_H
