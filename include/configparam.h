#ifndef YGZ_CONFIGPARAM_H_
#define YGZ_CONFIGPARAM_H_

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <thirdparty/sophus/sophus/se3.hpp>
#include <thirdparty/sophus/sophus/so3.hpp>
using cv::Mat;
using namespace Eigen;
using namespace Sophus;
namespace vill {

    class ConfigParam {
    public:
	static void setTbc(const SE3d &nTbc);
	
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ConfigParam(std::string configfile);

		static bool printConfig(std::ofstream & ofile);

        static bool GetUseIMUFlag() { return _bUseIMU; }
        
        static bool GetUseLaserPoseFlag() {return _bUseLaserpose;}
        
        static bool GetUseLaserMapFlag() {return _bUseLaserMap;}
        
        static bool GetUseCeres(){return _bUseCeres;}

        double _testDiscardTime;

        static SE3d GetSE3Tbc() { return _SE3Tbc; }

        static Matrix4d GetEigTbc();

        static Mat GetMatTbc();

        static Matrix4d GetEigT_cb();

        static Mat GetMatT_cb();

//         constexpr int GetLocalWindowSize();
		int GetLocalWindowSize();
		
		static int GetInitWindowSize();

        static double GetImageDelayToIMU();

        static bool GetAccMultiply9p8();

        static double GetG() { return _g; }
        
        static int GetLaserMapCirculate() {return laserMap_Circulate;}

        std::string _bagfile;
        std::string _imageTopic;
		std::string _imageRightTopic;
        std::string _imuTopic;
		std::string _laserposeTopic;

        static std::string getTmpFilePath();

        static std::string _tmpFilePath;

        static double GetVINSInitTime() { return _nVINSInitTime; }

		static int _verbose;
		static bool _bUseDirect;
		static bool _bUseIMU;
		static bool _bUseLaserpose;
		static bool _bUseLaserMap;
		static bool _bUseCeres; //else is g2o
		static int _minInitFeaturePoints;
		static int _minInitMapPoints;
		static int _minInitTrackPoints;
		static int _minTrackPoints;
		static int _minLocalMapTrackPoints;
		static double _scaleThreshold; 
		static double _gravityThreashold;
		static double _thetaThreashold;
		static int _paralaxThreashold;
		static double _cullingThreashold;
		
		int rows_, cols_;
		
// 		const static int _LocalWindowSize = 10;

    private:
        static SE3d _SE3Tbc;
        static Matrix4d _EigTbc;
        static Mat _MatTbc;
        static Matrix4d _EigTcb;
        static Mat _MatTcb;
        static int _LocalWindowSize;
		static int _InitWindowSize;
        static double _ImageDelayToIMU;
        static bool _bAccMultiply9p8;

        static double _g;
        static double _nVINSInitTime;
		
		static int laserMap_Circulate;

    };

    
    //for  stereo orbvio
    class StereoConfigParam : public  ConfigParam
    {
    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StereoConfigParam(std::string configfile);

    cv::Mat  _K_l, _K_r, _P_l, _P_r, _R_l, _R_r, _D_l, _D_r;
    int  _rows_l, _cols_l, _rows_r, _cols_r;

    private:
    

    };
}

#endif // CONFIGPARAM_H
