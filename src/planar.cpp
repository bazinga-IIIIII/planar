//============================================================================
// Name        : planar.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

//#include <iostream>
#include "common.h"
#include "rgbdframe.h"

using namespace std;
using namespace rgbd_tutor;
using namespace cv;

int main()
{
	cout << "start" <<endl;
    ParameterReader para;
    FrameReader     fr(para);
    RGBDFrame::Ptr old_frame(new RGBDFrame);
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera = para.getCamera();
    while( RGBDFrame::Ptr frame = fr.next())
    {
    	imshow("rgb", frame->rgb);
    	imshow("depth", frame->depth);
    	cv::waitKey(1);
    }
	cout << "end" << endl;
	return 0;
}
