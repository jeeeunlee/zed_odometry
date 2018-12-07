#ifndef POINT3D_H_
#define POINT3D_H_

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include "zed_param.h"

// using namespace std;
// using namespace cv;

struct Point3D{
    float X, Y, Z;    
};

class PointReconstructor{

public:
    PointReconstructor();

    // Copy constructor
    PointReconstructor(const PointReconstructor &point3d);

    // constructor
    PointReconstructor(const std::string &source);

    // compute for KeyPoint.
    Point3D compute(const cv::KeyPoint &kpleft, const cv::KeyPoint &kpright);

    // compute for Point2f.
    Point3D compute(const cv::Point2f &p2left, const cv::Point2f &p2right);

    // get Calibration information
    void getCalibInfo(const ZParam *param);
     

public:
    static bool mbInitialzedGetInformation;
    // static CALIBINFO LeftCalib, RightCalib;

    ZParam *mParam;
    cv::Mat Kl, Kr;
    cv::Mat HomoLtoR, HomoRtoL;
    
};

#endif /* POINT3D_H_ */