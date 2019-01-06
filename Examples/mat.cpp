//
// Created by kadn on 18-10-30.
//

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

using namespace cv;

int main()
{
    cv::Mat mat(4,2,CV_32F);
    cv::Mat mK(3,3,CV_32F);
    cv::Mat mDistCoef(4,1,CV_32F);
    mK.at<float>(0,0) = 500;
    mK.at<float>(1,1) = 500;
    mK.at<float>(2,2) = 1;
    mK.at<float>(0,2) = 319.5;
    mK.at<float>(1,2) = 239.5;

    mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
    mat.at<float>(1,0)=640; mat.at<float>(1,1)=0.0;
    mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=480;
    mat.at<float>(3,0)=640; mat.at<float>(3,1)=480;

    // Undistort corners
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);
    for(int i=0; i<mat.rows;i++)
        for(int j=0;j<mat.cols; j++)
                cout << mat.at<float>(i,j) << endl;
    return 0;
}
