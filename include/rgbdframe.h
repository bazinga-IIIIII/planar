/*
 * rgbdframe.h
 *
 *  Created on: Mar 15, 2016
 *      Author: wei
 */

#ifndef RGBDFRAME_H_
#define RGBDFRAME_H_


#include "common.h"
#include "parameter_reader.h"

//#include"Thirdparty/DBoW2/DBoW2/FORB.h"
//#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace rgbd_tutor{

//帧
class RGBDFrame
{
public:
    typedef shared_ptr<RGBDFrame> Ptr;

public:
    RGBDFrame() {}

public:
    // 数据成员
    int id  = -1;            //-1表示该帧不存在

    // 彩色图和深度图
    cv::Mat rgb, depth;
    // 该帧位姿
    // 定义方式为：x_local = T * x_world 注意也可以反着定义；
    //Eigen::Isometry3d       T=Eigen::Isometry3d::Identity();


    // 相机
    // 默认所有的帧都用一个相机模型
    CAMERA_INTRINSIC_PARAMETERS camera;

    // BoW回环特征
//    DBoW2::BowVector bowVec;

};

// FrameReader
// 从TUM数据集中读取数据的类
class FrameReader
{
public:
	vector<string>  rgbFiles, depthFiles;
	vector<string>  rgbTimes, depthTimes;
	void FrameWriter(RGBDFrame frame);
	FILE *fp;
    FrameReader( const rgbd_tutor::ParameterReader& para )
        : parameterReader( para )
    {
        init_tum( );
    	fp=fopen("/home/wei/workspace/slam/pose","r+");
    	if(fp == NULL)
    		cout << "Open file error!" << endl;
        ofstream out("/home/wei/workspace/slam/pose");
    }

    // 获得下一帧
    RGBDFrame::Ptr   next();

    // 重置index
    void    reset()
    {
        cout<<"重置 frame reader"<<endl;
        currentIndex = start_index;
    }

    // 根据index获得帧
    RGBDFrame::Ptr   get( const int& index )
    {
        if (index < 0 || index >= rgbFiles.size() )
            return nullptr;
        currentIndex = index;
        return next();
    }

protected:
    // 初始化tum数据集
    void    init_tum( );
protected:

    // 当前索引
    int currentIndex =0;
    // 起始索引
    int start_index  =0;

    const   ParameterReader&    parameterReader;

    // 文件名序列
//    vector<string>  rgbFiles, depthFiles;

    // 数据源
    string  dataset_dir;

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS     camera;
};

};



#endif /* RGBDFRAME_H_ */
