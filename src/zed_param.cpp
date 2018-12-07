#include "zed_param.h"

ZParam::ZParam(const std::string &source):mSource(source)
{
    initVariables();
}


void ZParam::initVariables()
{
    std::cout<< "initVariables"<<std::endl;
    cv::FileStorage fSettings(mSource, cv::FileStorage::READ);

    mCalibLeft.fx = fSettings["Camera.fx"];
    mCalibLeft.fy = fSettings["Camera.fy"];
    mCalibLeft.cx = fSettings["Camera.cx"];
    mCalibLeft.cy = fSettings["Camera.cy"];
    mCalibLeft.k1 = fSettings["Camera.k1"];
    mCalibLeft.k2 = fSettings["Camera.k2"];
    mCalibLeft.p1 = fSettings["Camera.p1"];
    mCalibLeft.p2 = fSettings["Camera.p2"];

    mCalibRight.fx = fSettings["Camera.rfx"];
    mCalibRight.fy = fSettings["Camera.rfy"];
    mCalibRight.cx = fSettings["Camera.rcx"];
    mCalibRight.cy = fSettings["Camera.rcy"];
    mCalibRight.k1 = fSettings["Camera.rk1"];
    mCalibRight.k2 = fSettings["Camera.rk2"];
    mCalibRight.p1 = fSettings["Camera.rp1"];
    mCalibRight.p2 = fSettings["Camera.rp2"];

    mwidth = fSettings["Camera.width"];
    mheight = fSettings["Camera.height"];   

    mBF = fSettings["Camera.bf"]; // Baseline times fx
    mCV = fSettings["Camera.CV"]; // RY
    mRX = fSettings["Camera.RX"]; // RX
    mRZ = fSettings["Camera.RZ"]; // RZ

    mBaseline = mBF/mCalibLeft.fx;
}