#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include "ros/package.h"
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp> 

#include "System.h"
#include "dc_optimizer.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

// #include "MessageViewer.h"
#include "opencv2/calib3d.hpp"


// #include "pluginlib/class_list_macros.h"

#include "dc_vmapping/depth_completion_list.h"
#include "dc_vmapping/image.h"

#include "glog/logging.h"


const double g3dm = 9.80665;
bool bAccMultiply98 = false;
cv::Mat M1l, M2l, M1r, M2r;
int counter = 0;

int cnt = 0;

void insert_RGB_image(dc_vmapping::depth_completion_list& srv, cv::Mat& cv_rgb_image){

	sensor_msgs::Image rgb_msg;
	cv_bridge::CvImage rgb_cvImage;
	rgb_cvImage.encoding = "bgr8";
	rgb_cvImage.header.frame_id = "/map";
	rgb_cvImage.header.seq = cnt;
	rgb_cvImage.header.stamp = ros::Time::now();
	rgb_cvImage.image = cv_rgb_image;
	rgb_msg = *rgb_cvImage.toImageMsg();
	
	
	dc_vmapping::image rgb_image;
	rgb_image.id = cnt;
	rgb_image.image = rgb_msg;
	
	srv.request.rgb_lists.push_back(rgb_image);
}

void insert_depth_image(dc_vmapping::depth_completion_list& srv, cv::Mat& cv_depth_image, int id){
	
		sensor_msgs::Image depth_msg;
		cv_bridge::CvImage depth_cvImage;
		depth_cvImage.encoding = "mono16";
		depth_cvImage.header.frame_id = "/map";
		depth_cvImage.header.seq = cnt;
		depth_cvImage.header.stamp = ros::Time::now();
		depth_cvImage.image = cv_depth_image;
		depth_msg = *depth_cvImage.toImageMsg();
		
		dc_vmapping::image depth_image;
		depth_image.id = id;
		depth_image.image = depth_msg;
		
		srv.request.depth_lists.push_back(depth_image);
		
}

void recover_gradients(dc_vmapping::depth_completion_list& srv, 
					   map<int,cv::Mat>& m_id_gradient, map<int,float>& m_id_loss){
	
	for(int i = 0; i < srv.response.gradient_lists.size(); i++){
		
		sensor_msgs::Image depth_msg = srv.response.gradient_lists[i].image;
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(depth_msg);
		m_id_gradient[srv.response.gradient_lists[i].id] = cv_ptr->image;
		std::cout << "gradient type "<<cv_ptr->image.type() <<" vs "<< CV_32FC1<< std::endl;
		m_id_loss[srv.response.gradient_lists[i].id] = srv.response.losses[i];
	}
	
}

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_colorlogtostderr=true;
	FLAGS_alsologtostderr = true;
	google::InstallFailureSignalHandler();
	ros::init(argc, argv, "stereo_sfm");
    ros::start();
	
	ros::NodeHandle nh("~");
	ros::NodeHandle n;
	
	vector<string > params;
	params.resize(6);
	int getParamNum = 1;
	params[0] = argv[0];
	params[1] = argv[1];
	params[2] = argv[2];
	params[3] = argv[3];
	params[4] = argv[4];
	params[5] = argv[5];
	
	cout <<"vocabulary " << params[1] << endl;
	cout <<"configFile " << params[2] << endl;
	cout << "dataFile" << params[3] << endl;
	cout <<"image_folder " << params[4] << endl;
	cout << "image_name" << params[5] << endl;
	
	if(getParamNum != 1)
    {
        cerr << endl << "Usage: rosrun vill StereoVILL path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
	
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
    vill::System SLAM(params[1],params[2], vill::System::STEREO,false);
	
	vill::StereoConfigParam config(params[2]);
	
	//check rectify
    if (config._K_l.empty() || config._K_r.empty() || config._P_l.empty() || config._P_r.empty() || config._R_l.empty() || config._R_r.empty() || config._D_l.empty() || config._D_r.empty() ||
            config._rows_l == 0 || config._rows_r == 0 || config._cols_l == 0 || config._cols_r == 0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }
    
	string base_image_folder = params[4];
	string image_file_name = params[5];
	std::cout <<"load image names from " << image_file_name << std::endl;
	
	string picPath = base_image_folder+"/images/";
	
	boost::filesystem::path dir(picPath);
	boost::filesystem::create_directory(dir);
	
	ifstream image_file(image_file_name);
	string image_name;
	
	
	
	ros::ServiceClient client_optimization = n.serviceClient<dc_vmapping::depth_completion_list>("depth_completion_server");  
	dc_vmapping::depth_completion_list srv;

	while( getline(image_file,image_name) && ros::ok()){
		
		cv::Mat left_image = cv::imread(base_image_folder+"/left/"+image_name);
		
		cv::Mat right_image = cv::imread(base_image_folder+"/right/"+image_name);
		
		string time = image_name;
		time.pop_back();	time.pop_back(); 	time.pop_back();	time.pop_back();
		std::cout << time << endl;
		
		cv::Mat left_image_crop = left_image.colRange(0,left_image.cols-25).rowRange(0,left_image.rows-24);
		cv::Mat right_image_crop = right_image.colRange(0,right_image.cols-25).rowRange(0,right_image.rows-24);
		
		SLAM.TrackStereo(left_image_crop,right_image_crop,atof(time.c_str()));
		
		while(!SLAM.CheckLocalMappingState()){
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		
		insert_RGB_image(srv, left_image_crop);
		
		char picName[100];
		sprintf(picName,"%010d.png",cnt++);
		cout <<"save image to "<< picPath+picName << endl;
		cv::imwrite(picPath+picName, left_image_crop);
		
	}
	

	std::map<int, cv::Mat> depth_images = SLAM.getDepthMap();
	std::map<int, cv::Mat>::iterator id_depth_iter = depth_images.begin();
	
	for(; id_depth_iter != depth_images.end(); id_depth_iter++){
		
		insert_depth_image(srv, id_depth_iter->second, id_depth_iter->first);

	}
	
	
	std::cout << "done with initialization" <<std::endl;
	std::cout << srv.request.depth_lists.size() <<std::endl;
	std::cout << srv.request.rgb_lists.size() <<std::endl;
	
	dc_optimizer DCoptimizer(SLAM.getMapPtr());
	
	double initial_res = DCoptimizer.optimize_BA_with_dc_error();
	
	std::cout <<"init res = "<< initial_res <<std::endl;
	
	
	if(client_optimization.call(srv)){
		std::cout << "receive service" << std::endl;
		
		map<int,cv::Mat> m_id_gradient;
		map<int,float> m_id_loss;
		recover_gradients(srv, m_id_gradient, m_id_loss);
		SLAM.update_gradient(m_id_gradient, m_id_loss);
		
		double initial_res = DCoptimizer.optimize_BA_with_dc_error();
	
		std::cout <<"res = "<< initial_res <<std::endl;
		
	}else{
		std::cerr<<"Failed to call service depth_completion_server"<<std::endl;
	}
	
	
// 	
// 	
// 	
// 	
// 	SLAM.SaveTrajectoryTUM(
// 		 params[3]+ "/FrameTrajectory_TUM_Format.txt");
// 	SLAM.saveData(params[3]);
// 	SLAM.saveDepthMap(params[3]);
// 	
	SLAM.Shutdown();
	ros::shutdown();
	
}
