#include "Converter.h"
#include <iostream>

namespace vill {

    PointMatcher< float >::TransformationParameters Converter::toPMTrans(const cv::Mat cvTranspose)
    {
	    PM::TransformationParameters T = PM::TransformationParameters::Identity(4, 4);
	    for(int i=0;i<4;i++)
		    for(int j=0; j<4; j++)
			    T(i,j) = cvTranspose.at<float>(i,j);
	    
	    return T;

    }

    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).
    Eigen::Vector3d Converter::rotationMatrixToEulerAngles(cv::Mat &R)
    {
    
    //     assert(cv::isRotationMatrix(R));

	float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );
    
	bool singular = sy < 1e-6; // If
    
	float x, y, z;
	if (!singular)
	{
	    x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
	    y = atan2(-R.at<float>(2,0), sy);
	    z = atan2(R.at<float>(1,0), R.at<float>(0,0));
	}
	else
	{
	    x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
	    y = atan2(-R.at<float>(2,0), sy);
	    z = 0;
	}
	return Eigen::Vector3d(x, y, z);
	
	
	
    }
  
   

    std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors) {
        std::vector<cv::Mat> vDesc;
        vDesc.reserve(Descriptors.rows);
        for (int j = 0; j < Descriptors.rows; j++)
            vDesc.push_back(Descriptors.row(j));

        return vDesc;
    }
    
    cv::Mat Converter::toCvMat(const SE3d &SE3) {
        return toCvMat(SE3.matrix());
    }

    Mat Converter::toCvMat(const SE3f &SE3) {
        return toCvMat(SE3.matrix());
    }
   
    cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m) {
        cv::Mat cvMat(4, 4, CV_32F);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                cvMat.at<float>(i, j) = m(i, j);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 4, 4> &m) {
        cv::Mat cvMat(4, 4, CV_32F);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                cvMat.at<float>(i, j) = m(i, j);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m) {
        cv::Mat cvMat(3, 3, CV_32F);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cvMat.at<float>(i, j) = m(i, j);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix3f &m) {
        cv::Mat cvMat(3, 3, CV_32F);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cvMat.at<float>(i, j) = m(i, j);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m) {
        cv::Mat cvMat(3, 1, CV_32F);
        for (int i = 0; i < 3; i++)
            cvMat.at<float>(i) = m(i);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 1> &m) {
        cv::Mat cvMat(3, 1, CV_32F);
        for (int i = 0; i < 3; i++)
            cvMat.at<float>(i) = m(i);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t) {
        cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cvMat.at<float>(i, j) = R(i, j);
            }
        }
        for (int i = 0; i < 3; i++) {
            cvMat.at<float>(i, 3) = t(i);
        }

        return cvMat.clone();
    }

    Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cvVector) {
        Eigen::Matrix<double, 3, 1> v;
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

        return v;
    }

    Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cvPoint) {
        Eigen::Matrix<double, 3, 1> v;
        v << cvPoint.x, cvPoint.y, cvPoint.z;

        return v;
    }

    Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cvMat3) {
        Eigen::Matrix<double, 3, 3> M;

        M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
                cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
                cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

        return M;
    }

    std::vector<float> Converter::toQuaternion(const cv::Mat &M) {
        Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
        Eigen::Quaterniond q(eigMat);

        std::vector<float> v(4);
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();

        return v;
    }

} //namespace ORB_SLAM
