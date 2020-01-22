#include "configparam.h"
#include <iostream>
#include <fstream>
// #include "Optimizer_g2o.h"
namespace vill {
    double ConfigParam::_g = 9.810;
	int ConfigParam::laserMap_Circulate = 5;

    Sophus::SE3d ConfigParam::_SE3Tbc = Sophus::SE3d();
    Eigen::Matrix4d ConfigParam::_EigTbc = Eigen::Matrix4d::Identity();
    cv::Mat ConfigParam::_MatTbc = cv::Mat::eye(4, 4, CV_32F);
    Eigen::Matrix4d ConfigParam::_EigTcb = Eigen::Matrix4d::Identity();
    cv::Mat ConfigParam::_MatTcb = cv::Mat::eye(4, 4, CV_32F);
    double ConfigParam::_ImageDelayToIMU = 0;
    bool ConfigParam::_bAccMultiply9p8 = false;
    double ConfigParam::_scaleThreshold = 0.25;
    double ConfigParam::_gravityThreashold = 0.025;
    double ConfigParam::_thetaThreashold = 0.25;
    int ConfigParam::_paralaxThreashold = 200;
    double ConfigParam::_cullingThreashold = 0.95;

    std::string ConfigParam::_tmpFilePath = "F:/AR3D/ORB-YGZ-SLAM-master/log/";
	int ConfigParam::_verbose = 1;
	bool ConfigParam::_bUseDirect = false;
	bool ConfigParam::_bUseIMU = false;
	bool ConfigParam::_bUseLaserpose = false;
	bool ConfigParam::_bUseLaserMap = false;
	bool ConfigParam::_bUseCeres = false;
	
	int ConfigParam::_LocalWindowSize = 10;
	int ConfigParam::_InitWindowSize = 2;
	int ConfigParam::_minInitFeaturePoints = 100;
	int ConfigParam::_minInitMapPoints = 100;
	int ConfigParam::_minInitTrackPoints = 100;
	int ConfigParam::_minTrackPoints = 10;
	int ConfigParam::_minLocalMapTrackPoints = 30;
	

	double ConfigParam::_nVINSInitTime = 15;
	
    ConfigParam::ConfigParam(std::string configfile) {
        cv::FileStorage fSettings(configfile, cv::FileStorage::READ);
		/// configuration
		int tmpbUseDirect = fSettings["bUseDirect"];
		_bUseDirect = (tmpbUseDirect != 0);

		int tmpbUseIMU = fSettings["bUseIMU"];
		_bUseIMU = (tmpbUseIMU != 0);
		
		int tmpbUseLaser = fSettings["bUseLaser"];
		_bUseLaserpose = (tmpbUseLaser != 0);
		
		int tmpbUseLaserMap = fSettings["bUseLaserMap"];
		_bUseLaserMap = (tmpbUseLaserMap != 0);
		
		int tmpbUseCeres = fSettings["bUseCeres"];
		_bUseCeres = (tmpbUseCeres != 0);

		int tmpbVerbose = fSettings["bVerbose"];
		_verbose = (tmpbVerbose != 0);

		fSettings["InitVIOTmpPath"] >> _tmpFilePath;
		
		fSettings["imutopic"] >> _imuTopic;
		fSettings["laserposetopic"] >> _laserposeTopic;
		fSettings["imagetopic"] >> _imageTopic;
		fSettings["rimagetopic"] >> _imageRightTopic;
		std::cout<<"imu topic: "<<_imuTopic<<std::endl;
		std::cout<<"laser pose topic: " << _laserposeTopic << std::endl;
		std::cout<<"image topic: "<<_imageTopic<<std::endl;
		std::cout << "right image topic: " << _imageRightTopic << std::endl; 

		/// parameters
		_LocalWindowSize = fSettings["LocalMapping.LocalWindowSize"];
		
		_InitWindowSize = fSettings["LocalMapping.InitWindowSize"];

		_minInitFeaturePoints = fSettings["visual.minInitFeaturePoints"];

		_minInitMapPoints = fSettings["visual.minInitMapPoints"];

		_minInitTrackPoints = fSettings["visual.minInitTrackPoints"];

		_minTrackPoints = fSettings["visual.minTrackPoints"];

		_minLocalMapTrackPoints = fSettings["visual.minLocalMapTrackPoints"];


		_nVINSInitTime = fSettings["vins.InitTime"];

		_ImageDelayToIMU = fSettings["Camera.delaytoimu"];

		int tmpBool = fSettings["IMU.multiplyG"];
		_bAccMultiply9p8 = (tmpBool != 0);
		
		_scaleThreshold = fSettings["Init.ScaleThreashold"];
		_gravityThreashold = fSettings["Init.GravityThreashold"];
		_thetaThreashold = fSettings["Init.ThetaThreashold"];
		_paralaxThreashold = fSettings["Init.ParalaxThreashold"];
		_cullingThreashold = fSettings["Init.CullingThreashold"];
		
		laserMap_Circulate = fSettings["laserMapCirculate"];
		
		cols_ = fSettings["Camera.width"];
		rows_ = fSettings["Camera.height"];

        {
            cv::FileNode Tbc_ = fSettings["Camera.Tbc"];
            Eigen::Matrix<double, 3, 3> R;
            R << Tbc_[0], Tbc_[1], Tbc_[2],
                    Tbc_[4], Tbc_[5], Tbc_[6],
                    Tbc_[8], Tbc_[9], Tbc_[10];
            Eigen::Quaterniond qr(R);
            R = qr.normalized().toRotationMatrix();
            Eigen::Matrix<double, 3, 1> t(Tbc_[3], Tbc_[7], Tbc_[11]);
            _EigTbc = Eigen::Matrix4d::Identity();
            _EigTbc.block<3, 3>(0, 0) = R;
            _EigTbc.block<3, 1>(0, 3) = t;
            _MatTbc = cv::Mat::eye(4, 4, CV_32F);
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    _MatTbc.at<float>(i, j) = _EigTbc(i, j);

            _SE3Tbc = Sophus::SE3(R, t);

            _EigTcb = Eigen::Matrix4d::Identity();
            _EigTcb.block<3, 3>(0, 0) = R.transpose();
            _EigTcb.block<3, 1>(0, 3) = -R.transpose() * t;
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    _MatTcb.at<float>(i, j) = _EigTcb(i, j);
        }

        cout <<"Tbc\n"<<_SE3Tbc.matrix()<<endl;

    }
    
    void ConfigParam::setTbc(const SE3d& nTbc)
    {
	_SE3Tbc = nTbc;
	Matrix3d R = nTbc.rotationMatrix();
	Vector3d t = nTbc.translation();
	_EigTbc.block<3, 3>(0, 0) = R;
	_EigTbc.block<3, 1>(0, 3) = t;
	for (int i = 0; i < 4; i++)
	    for (int j = 0; j < 4; j++)
		_MatTbc.at<float>(i, j) = _EigTbc(i, j);
	_EigTcb = Eigen::Matrix4d::Identity();
	_EigTcb.block<3, 3>(0, 0) = R.transpose();
	_EigTcb.block<3, 1>(0, 3) = -R.transpose() * t;
	for (int i = 0; i < 4; i++)
	    for (int j = 0; j < 4; j++)
		_MatTcb.at<float>(i, j) = _EigTcb(i, j);
    }
    
    std::string ConfigParam::getTmpFilePath() {
        return _tmpFilePath;
    }

    Eigen::Matrix4d ConfigParam::GetEigTbc() {
        return _EigTbc;
    }

    cv::Mat ConfigParam::GetMatTbc() {
        return _MatTbc.clone();
    }

    Eigen::Matrix4d ConfigParam::GetEigT_cb() {
        return _EigTcb;
    }

    cv::Mat ConfigParam::GetMatT_cb() {
        return _MatTcb.clone();
    }

    int  ConfigParam::GetLocalWindowSize() {
        return _LocalWindowSize;
    }
    
    int ConfigParam::GetInitWindowSize(){
	return _InitWindowSize;
    }

    double ConfigParam::GetImageDelayToIMU() {
        return _ImageDelayToIMU;
    }

    bool ConfigParam::GetAccMultiply9p8() {
        return _bAccMultiply9p8;
    }

	bool ConfigParam::printConfig(std::ofstream & ofile){
		if (!ofile)
			return false;
		/// config 
		ofile << "configuration: " << std::endl;
		ofile << "whether use Direct 0/1: " << _bUseDirect << std::endl;
		ofile << "whether use IMU 0/1: " << _bUseIMU << std::endl;
		ofile << "whether use laserpose 0/1: " << _bUseLaserpose << std::endl;
		ofile << "whether verbose 0/1: " << _verbose << std::endl;
		ofile << "save tmp file in " << _tmpFilePath << std::endl;

		/// parameters
		ofile << std::endl << "parameters: " << std::endl;
		ofile << "local window size: " << _LocalWindowSize << std::endl;
		ofile << "min init map points: " << _minInitMapPoints << endl;
		ofile << "min init track points: " << _minInitTrackPoints << endl;
		ofile << "VINS initialize time: " << _nVINSInitTime << std::endl;
		ofile << "timestamp image delay to imu: " << _ImageDelayToIMU << std::endl;
		ofile << "whether acc*9.8? 0/1: " << _bAccMultiply9p8 << std::endl;
		ofile << "Tbc inited:" << std::endl << _EigTbc << std::endl << _MatTbc << std::endl;
		return true;
	}
	
	//派生类构造函数
	StereoConfigParam::StereoConfigParam(std::string configfile):ConfigParam(configfile)
	{
	    cv::FileStorage fsSettings(configfile, cv::FileStorage::READ);
	    fsSettings["LEFT.K"] >> _K_l;
	    fsSettings["RIGHT.K"] >> _K_r;

	    fsSettings["LEFT.P"] >> _P_l;
	    fsSettings["RIGHT.P"] >> _P_r;

	    fsSettings["LEFT.R"] >> _R_l;
	    fsSettings["RIGHT.R"] >> _R_r;

	    fsSettings["LEFT.D"] >> _D_l;
	    fsSettings["RIGHT.D"] >> _D_r;

	    _rows_l = fsSettings["LEFT.height"];
	    _cols_l = fsSettings["LEFT.width"];
	    _rows_r = fsSettings["RIGHT.height"];
	    _cols_r = fsSettings["RIGHT.width"];
		
// 		Optimizer_g2o::distTH = fsSettings["Optimizer.distTH"];
// 		Optimizer_g2o::invariatDist = fsSettings["Optimizer.invariatDist"];
// 		Optimizer_g2o::searchN = fsSettings["Optimizer.searchN"];
// 		Optimizer_g2o::iteration_count = fsSettings["Optimizer.iteration_count"];
// 		Optimizer_g2o::p2pInfo = fsSettings["Optimizer.p2pInfo"];
// 		Optimizer_g2o::p2planeInfo = fsSettings["Optimizer.p2planeInfo"];
// 		Optimizer_g2o::relativeInfo = fsSettings["Optimizer.relativeInfo"];
// 		Optimizer_g2o::relativeReproInfo = fsSettings["Optimizer.relativeReproInfo"];
// 		Optimizer_g2o::delta = fsSettings["Optimizer.delta"];
// 		Optimizer_g2o::ceres_iteration_times = fsSettings["Opt.iteration_times"];
// 		Optimizer_g2o::ceres_th_laser = fsSettings["Opt.ThLaser"];
// 		Optimizer_g2o::ceres_th_preintegration = fsSettings["Opt.ThPreintegration"];
// 		Optimizer_g2o::ceres_th_reprojection = fsSettings["Opt.ThReprojection"];
// 		Optimizer_g2o::prioriInfo = fsSettings["Optimizer.prioriInfo"];
// 		Optimizer_g2o::ceres_th_positionUp = fsSettings["Opt.ThPositionUp"];
// 		Optimizer_g2o::ceres_th_velocityUp = fsSettings["Opt.ThVelocityUp"];
// 		Optimizer_g2o::ceres_th_rotationUp = fsSettings["Opt.ThRotationUp"];
// 		Optimizer_g2o::ceres_th_biasaUp = fsSettings["Opt.ThBiasa"];
// 		Optimizer_g2o::ceres_th_biasgUp = fsSettings["Opt.ThBiasg"];
// 		Optimizer_g2o::ceres_th_prioriUp = fsSettings["Opt.ThPriori"];
// 		Optimizer_g2o::th_iter_icp_fail = fsSettings["Opt.th_it_icp_fail"];
	}

}
