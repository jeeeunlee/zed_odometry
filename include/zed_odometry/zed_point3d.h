#ifndef POINT3D_H_
#define POINT3D_H_

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include "zed_param.h"
#include "zed_common.h"

// using namespace std;
// using namespace cv;

struct Point3D{
    float X, Y, Z;    
};

class PointReconstructor{

public:
    PointReconstructor();

    // Copy constructor
    // PointReconstructor(const PointReconstructor &point3d);

    // constructor
    PointReconstructor(ZParam *param);

    // estimate camera pose from 3d point projection
    void solvePnP(const V_POINT3F &keypoints3d, const V_POINT2F &keypoints2d, bool bleft=true);

    // compute for KeyPoint
    void compute(const V_POINT2F &kpleft, const V_POINT2F &kpright, V_POINT3F &kp3f);

    // compute undistortKeyPoint
    V_POINT2F UndistortKeyPoints(const V_POINT2F &mvKeys, const cv::Mat &K, const cv::Mat &distCoeff);

    // reproject 3D point to 2D image
    void reprojection(const V_POINT3F &kp3f, V_POINT2F &reproj_kpleft, V_POINT2F &reproj_kpright);

    // get Calibration information
    void getCalibInfo();

    void printMatrix(const std::string &caption, const cv::Mat &MAT);
    void save(const V_POINT2F &p,  const char *filename, std::_Ios_Openmode type = std::ios::out);
    void save(const V_POINT3F &p,  const char *filename, std::_Ios_Openmode type = std::ios::out);
    void save(const cv::Mat &p,  const char *filename, std::_Ios_Openmode type = std::ios::out);
     

public:
    static bool mbInitialzedGetInformation;
    // static CALIBINFO LeftCalib, RightCalib;

    ZParam *mParam;
    cv::Mat mKL, mKR;
    cv::Mat mDistCoeffsL, mDistCoeffsR;
    cv::Mat mHomoLtoR, mHomoRtoL;
    
};

#endif /* POINT3D_H_ */