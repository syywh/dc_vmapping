#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include "cereal/archives/binary.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/map.hpp"

#include "KeyFrame.h"
namespace cereal{
	
template<class Archive>
void save(Archive& ar, const cv::KeyPoint& m){
	ar(m.pt.x,m.pt.y, m.angle, m.octave);
}

template<class Archive>
void load(Archive& ar,  cv::KeyPoint& m){
	ar(m.pt.x,m.pt.y, m.angle, m.octave);
}

// 				   C1   C2   C3    C4
// #define CV_8U   0    8    16    24
// #define CV_8S   1    9	
// #define CV_16U  2
// #define CV_16S  3
// #define CV_32S  4
// #define CV_32F  5
// #define CV_64F  6	14
template<class Archive>
void save(Archive& ar, const cv::Mat& m){
	ar(m.rows, m.cols, m.type());
	
	int size = m.type();
	int type_size = 0;
	switch (size){
		case 0:
			type_size = sizeof(uchar);
			break;
		case 5:
			type_size = 4*sizeof(uchar);
			break;
		case 6:
			type_size = 8*sizeof(uchar);
			break;
		case 16:
			type_size = 3*sizeof(uchar);
			break;
		default:
			std::cerr<<"No registed cv::Mat type "<<m.type() <<std::endl;
			
	}
	ar(binary_data(m.data, m.rows * m.cols * type_size));
}

template<class Archive>
void load(Archive& ar, cv::Mat& m){
	int row, col,type;
	ar(row, col, type);

	int type_size = 0;
	switch (type){
		case 0:
			type_size = sizeof(uchar);
			break;
		case 5:
			type_size = 4*sizeof(uchar);
			break;
		case 6:
			type_size = 8*sizeof(uchar);
			break;
		case 16:
			type_size = 3*sizeof(uchar);
			break;
		default:
			std::cerr<<"No registed cv::Mat type " << type <<std::endl;
			
	}	
	m.create(row, col, type);
	ar(binary_data(m.data, m.rows*m.cols*type_size));
}
	
	
template<class Archive>
void save(Archive& ar, const Eigen::Matrix3f& m){
	ar(binary_data(m.data(), 9 * sizeof(float)));
}

template<class Archive>
void load(Archive& ar,  Eigen::Matrix3f& m){
	ar(binary_data(m.data(), 9 * sizeof(float)));
}

template<class Archive>
void save(Archive& ar, const Eigen::Matrix4f& m){
	ar(binary_data(m.data(), 16 * sizeof(float)));
}

template<class Archive>
void load(Archive& ar,  Eigen::Matrix4f& m){
	ar(binary_data(m.data(), 16 * sizeof(float)));
}

template<class Archive>
void save(Archive& ar, const Eigen::Vector3f& m){
	ar(binary_data(m.data(), 3 * sizeof(float)));
}

template<class Archive>
void load(Archive& ar,  Eigen::Vector3f& m){
	ar(binary_data(m.data(), 3 * sizeof(float)));
}

	
}

#endif