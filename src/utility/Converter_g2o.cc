#include "Converter_g2o.h"
#include "Converter.h"
namespace vill {
	g2o::SE3Quat Converter_g2o::toSE3Quat(const cv::Mat &cvT) {
		Eigen::Matrix<double, 3, 3> R;
		R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
			cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2),
			cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2);

		Eigen::Matrix<double, 3, 1> t(cvT.at<float>(0, 3), cvT.at<float>(1, 3), cvT.at<float>(2, 3));

		return g2o::SE3Quat(R, t);
	}
	
	g2o::SE3Quat Converter_g2o::toSE3Quat(const SE3d &T) {
		Eigen::Matrix<double, 3, 3> R = T.rotationMatrix();

		Eigen::Matrix<double, 3, 1> t = T.translation();

		return g2o::SE3Quat(R, t);
	}
	
	cv::Mat Converter_g2o::toCvMat(const g2o::SE3Quat &SE3) {
		Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
		return Converter::toCvMat(eigMat);
	}

	cv::Mat Converter_g2o::toCvMat(const g2o::Sim3 &Sim3) {
		Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
		Eigen::Vector3d eigt = Sim3.translation();
		double s = Sim3.scale();
		return Converter::toCvSE3(s * eigR, eigt);
	}
	
	Sophus::SE3d Converter_g2o::QuattoSE3d(const g2o::SE3Quat& gSim3)
	{
		Sophus::SE3d Tse3;
		Tse3.setRotationMatrix(gSim3.rotation().matrix());
		Tse3.translation() = gSim3.translation();
		return Tse3;

	}
	
	MatrixXd Converter_g2o::getBlockMatrix(const MatrixXd& originMat, int row, int col, int begin_row, int begin_col)
	{
		Eigen::MatrixXd newMat(row, col);
		
		for(int i = begin_row; i < begin_row+row; i++){
			for(int j = begin_col; j < begin_col+col; j++){
				newMat(i-begin_row, j-begin_col) = originMat(i,j);
			}
		}
		
		return newMat;

	}


}