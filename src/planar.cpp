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
#include <sys/time.h>


#define DEPTH_HEIGHT 480
#define DEPTH_WIDTH  640
#define DEPTH_VISION_CENTER_X 320
#define DEPTH_VISION_CENTER_Y 240
const float DEPTH_VISION_Z_DATA2METER_P = 1.0;
const double DEPTH_VISION_ZX_DATA2METER_P = 0.001512;
const double DEPTH_VISION_ZY_DATA2METER_P = 0.001512;

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

    struct timeval start, stop, diff;
    gettimeofday(&start, 0);

//        Mat src = imread("d2.bmp");
 //       Mat src = imread("1.png");
        int i, j;
        //计算空间xyz
        vector<Vec3f> original_3D_point(DEPTH_WIDTH*DEPTH_HEIGHT);
        for (i = 0; i<DEPTH_WIDTH; i++)
        {
            for (j = 0; j<DEPTH_HEIGHT; j++)
            {
                float x, y, z;
                y = double(frame->depth.ptr<ushort>(j)[i]) / camera.scale;
                //y = double(src.ptr<ushort>(j)[i]) / camera.scale;
                x = (i - camera.cx) * y / camera.fx;
                z = (j - camera.cy) * y / camera.fy;
                //y = src.at<Vec3b>(j, i)[0] + 256 * src.at<Vec3b>(j, i)[1] + 65536 * src.at<Vec3b>(j, i)[2];
                //x = float(y * DEPTH_VISION_ZX_DATA2METER_P * (i - DEPTH_VISION_CENTER_X));
                //z = float(y * DEPTH_VISION_ZY_DATA2METER_P * (j - DEPTH_VISION_CENTER_Y));
                original_3D_point[j*DEPTH_WIDTH + i] = Vec3f(x, y, z);
            }
        }


        //计算切线
        vector<Vec3f> tan_vec_x(DEPTH_WIDTH*DEPTH_HEIGHT);
        vector<Vec3f> tan_vec_y(DEPTH_WIDTH*DEPTH_HEIGHT);
        for (i = 2; i<DEPTH_WIDTH - 2; i++)
        {
            for (j = 2; j<DEPTH_HEIGHT - 2; j++)
            {
                tan_vec_x[j*DEPTH_WIDTH + i] = (original_3D_point[j*DEPTH_WIDTH + i + 2] - original_3D_point[j*DEPTH_WIDTH + i - 2]) / 2;
                tan_vec_y[j*DEPTH_WIDTH + i] = (original_3D_point[(j + 2)*DEPTH_WIDTH + i] - original_3D_point[(j - 2)*DEPTH_WIDTH + i]) / 2;
            }
        }
        for (i = 0; i<DEPTH_WIDTH; i++)
        {
            tan_vec_x[i] = Vec3f(0, 0, 0);
            tan_vec_x[i + DEPTH_HEIGHT] = Vec3f(0, 0, 0);
            tan_vec_x[DEPTH_WIDTH*(DEPTH_HEIGHT - 1) + i] = Vec3f(0, 0, 0);
            tan_vec_x[DEPTH_WIDTH*(DEPTH_HEIGHT - 2) + i] = Vec3f(0, 0, 0);
        }
        for (j = 1; j<DEPTH_HEIGHT - 1; j++)
        {
            tan_vec_y[j*DEPTH_WIDTH - 1] = Vec3f(0, 0, 0);
            tan_vec_x[j*DEPTH_WIDTH - 2] = Vec3f(0, 0, 0);
            tan_vec_y[j*DEPTH_WIDTH] = Vec3f(0, 0, 0);
            tan_vec_x[j*DEPTH_WIDTH + 1] = Vec3f(0, 0, 0);
        }


        //滤波
        Mat a1 = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32FC1);
        for (i = 0; i<DEPTH_WIDTH; i++)
        {
        	for (j = 0; j<DEPTH_HEIGHT; j++)
            {
                a1.at<float>(j, i) = tan_vec_x[j*DEPTH_WIDTH + i](0);
            }
        }
        GaussianBlur(a1, a1, Size(3, 3), 0, 0);
        for (i = 0; i<DEPTH_WIDTH; i++)
        {
        	for (j = 0; j<DEPTH_HEIGHT; j++)
            {
                tan_vec_x[j*DEPTH_WIDTH + i](0) = a1.at<float>(j, i);
            }
        }


        //计算法线
        vector<Vec3f> nor_vec_n(DEPTH_WIDTH*DEPTH_HEIGHT);
        float tempx, tempy, tempz, pos;
        for (i = 0; i<DEPTH_WIDTH; i++)
        {
            for (j = 0; j<DEPTH_HEIGHT; j++)
            {
                pos = j*DEPTH_WIDTH + i;
                tempx = tan_vec_x[pos](1)*tan_vec_y[pos](2) - tan_vec_x[pos](2)*tan_vec_y[pos](1);
                tempy = tan_vec_x[pos](2)*tan_vec_y[pos](0) - tan_vec_x[pos](0)*tan_vec_y[pos](2);
                tempz = tan_vec_x[pos](0)*tan_vec_y[pos](1) - tan_vec_x[pos](1)*tan_vec_y[pos](0);
                nor_vec_n[pos] = Vec3f(tempx, tempy, tempz);
            }
        }


        //归一化
        float mold;
        for (i = 0; i<DEPTH_WIDTH; i++)
        {
            for (j = 0; j<DEPTH_HEIGHT; j++)
            {
                pos = j*DEPTH_WIDTH + i;
                mold = sqrt(pow(nor_vec_n[pos](0), 2) + pow(nor_vec_n[pos](1), 2) + pow(nor_vec_n[pos](2), 2));
                nor_vec_n[pos] = Vec3f(nor_vec_n[pos](0) / mold, nor_vec_n[pos](1) / mold, nor_vec_n[pos](2) / mold);
                //cout<<mold<<"  "<<nor_vec_n.at(pos)[0]<<"  "<<nor_vec_n.at(pos)[1]<<"  "<<nor_vec_n.at(pos)[2]<<endl;
            }
        }


        gettimeofday(&diff, 0);


        //image segmentation and clustering
        //threshold1,threshold2
        vector<int> temp1;//���ڴ洢�����ĵ��λ��
        vector<int> temp2;//���ڴ洢temp1�ж�Ӧλ�õĵ�����
        vector<float> z_buff;
        vector<int>::iterator iter;
        ////////////////////////////////////////////////////////////////////////////////////////////
        float threshold1 = 0.7;//0.85;
        ////////////////////////////////////////////////////////////////////////////////////////////
        int length = 0;
        j = 0;
        //cout<<"start"<<endl;
        for (i = 0; i<DEPTH_HEIGHT*DEPTH_WIDTH; i++)
        {

            if (abs(nor_vec_n[i](2)) > threshold1)
            {	//cout<<nor_vec_n[i](2)<<"  ";
                temp1.push_back(i);
                temp2.push_back(0);
                j = original_3D_point[i](2);
                z_buff.push_back(j);
            }
        }
        i = 0;
        j = 0;
        Mat image1(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);//������ʾ����z����������һ����ֵ�ĵ�
        Mat image2(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);//������ʾ�ָ���
        Mat image3 = cv::Mat::zeros(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC3);//������ʾ�ָ���
        //CvMat *image3=cvCreateMat(DEPTH_HEIGHT,DEPTH_WIDTH,CV_8UC3);//������ʾ�ָ���
        for (iter = temp1.begin(); iter != temp1.end(); iter++)
        {
            i = (*iter) % DEPTH_WIDTH;
            j = (*iter) / DEPTH_WIDTH;
            image1.at<uchar>(j, i) = 255;
        }
        //namedWindow("image1");
        imshow("image1",image1);



        length = temp1.size();
        //vector<int> leibie;
        vector<int> shuliang;
        vector<float> junzhi;
        ////////////////////////////////////////////////////////////////////////////////////////////
        float threshold2 = 55;
        ////////////////////////////////////////////////////////////////////////////////////////////
        int add_flag;
        //leibie.push_back(0);//????

        shuliang.push_back(1);
        junzhi.push_back(z_buff[0]);
        temp2[0] = 1;
        //cvWaitKey(0);
        //return 0;


        //cout<<"aaa"<<z_buff[0]<<endl;
        //cout<<"aaa"<<temp2.size()<<endl;
        for (i = 1; i<length; i++)
        {
            add_flag = 1;
            //cout<<z_buff[i]<<"  ";
            for (j = 0; j<shuliang.size(); j++)
            {
                if (abs(z_buff[i] - junzhi[j]) < threshold2)
                {
                    shuliang[j] = shuliang[j] + 1;
                    junzhi[j] = (junzhi[j] * (shuliang[j] - 1) + z_buff[i]) / shuliang[j];
                    temp2[i] = j + 1;
                    add_flag = 0;
                    break;
                }
            }
            if (add_flag)
            {
                if (j == 107)
                {
                    cout << "i am here" << endl;
                }
                temp2[i] = j + 1;
                shuliang.push_back(1);
                junzhi.push_back(z_buff[i]);
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////
        int threshold3 = 8000;
        ////////////////////////////////////////////////////////////////////////////////////////////
        //cout<<"hello  "<<shuliang.size()<<endl;
        //cout<<"length "<<length<<endl;
        //for (iter=shuliang.begin();iter!=shuliang.end();iter++)
        for (j = 0; j<shuliang.size(); j++)
        {
            if (shuliang[j] < threshold3)
            {
                for (i = 0; i<length; i++)
                {
                    if (temp2[i] == (j + 1))
                    {
                        temp2[i] = 0;
                    }
                }
            }
            else
            {
                cout << j << "   " << shuliang[j] << endl;
            }
        }
        //for (iter=temp1.begin();iter!=temp1.end();iter++)
        //{
        //	i=(*iter)%DEPTH_WIDTH;
        //	j=(*iter)/DEPTH_WIDTH;
        //	image2.at<uchar>(j,i)=255;
        //}
        //cvSetZero(image3);
        for (i = 0; i<length; i++)
        {
            //image2.at<uchar>(temp1[i]/DEPTH_WIDTH,temp1[i]%DEPTH_WIDTH)=255;
            if (temp2[i])
            {
                //cout<<temp2[i]<<" ";//<<i<<endl;
                image2.at<uchar>(temp1[i] / DEPTH_WIDTH, temp1[i] % DEPTH_WIDTH) = (temp2[i] * 17) % 256;
                //image3[temp1[i]/DEPTH_WIDTH][temp1[i]%DEPTH_WIDTH]=Vec3b((i*30%200)+55,(i*50%200)+55,(i*90%200)+40);
                image3.at<Vec3b>(temp1[i] / DEPTH_WIDTH, temp1[i] % DEPTH_WIDTH) = Vec3b((temp2[i] * 20 % 200) + 55, (temp2[i] * 50 % 200) + 55, (temp2[i] * 100 % 200) + 40);
            }
        }
        //namedWindow("image2");
        //imshow("image2",image2);
        namedWindow("image3");
        imshow("image3", image3);
        //cvnamedWindow("image3");
        //imshow("image3",image3);

        gettimeofday(&stop, 0);
        //tim_subtract(&diff, &start, &stop);
        cout << start.tv_sec << endl;
        cout << start.tv_usec << endl;
        cout << diff.tv_sec << endl;
        cout << diff.tv_usec << endl;
        cout << stop.tv_sec << endl;
        cout << stop.tv_usec << endl;

        //waitKey(0);
    }
    cout << "end" << endl;
    return 0;
}
