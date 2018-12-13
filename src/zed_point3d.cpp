#include "zed_point3d.h"

bool PointReconstructor::mbInitialzedGetInformation=true;

PointReconstructor::PointReconstructor()
{}

PointReconstructor::PointReconstructor(ZParam *param)
{
    if(mbInitialzedGetInformation)
    {
        mParam = param;           
        getCalibInfo();
        mbInitialzedGetInformation=false;
    }

}

// Constructor for KeyPoint.
void PointReconstructor::compute(const std::vector<cv::Point2f> &kpleft, const std::vector<cv::Point2f> &kpright, std::vector<cv::Point3f> &kp3f)
{
    cv::Mat projMatr1 = cv::Mat::eye(3,4,CV_32F), projMatr2 = cv::Mat::eye(3,4,CV_32F);

    projMatr1 = mKL*projMatr1;
    projMatr2 = mKR*mHomoRtoL;     
    
    cv::Mat output(4,kpleft.size(),CV_32F);

    cv::triangulatePoints(projMatr1, projMatr2, kpleft, kpright, output);

    cv::Point3f point3f;
    // std::vector<cv::Point3f> vpoint3f;
    kp3f.clear();
    cv::Mat p3f;
    for(uint i = 0; i < output.cols; i++) 
    {
        cv::Mat _p3h = output.col(i);
        convertPointsFromHomogeneous(_p3h.t(), p3f);
        point3f.x = p3f.at<float>(0);
        point3f.y = p3f.at<float>(1);
        point3f.z = p3f.at<float>(2);

        kp3f.push_back(point3f);
    }
}


std::vector<cv::KeyPoint> PointReconstructor::UndistortKeyPoints(const std::vector<cv::KeyPoint> &mvKeys, const CALIBINFO &calibinfo, const cv::Mat &distCoeff)
{
    std::vector<cv::KeyPoint> mvKeysUn;
    cv::KeyPoint kp;
    int N = mvKeys.size();

    // no distortion -> return original keyPoint
    if(distCoeff.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return mvKeysUn;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F), matR(N,2,CV_32F);
    for(int i=0; i< mvKeys.size(); i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // camera metrix
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = calibinfo.fx;
    K.at<float>(1,1) = calibinfo.fy;
    K.at<float>(0,2) = calibinfo.cx;
    K.at<float>(1,2) = calibinfo.cy;

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,K,distCoeff,cv::Mat(),K);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }

    return mvKeysUn;
}


void PointReconstructor::getCalibInfo()
{
    // CAMERA MATRIX
    mKL = cv::Mat::eye(3,3,CV_32F);    
    mKL.at<float>(0,0) = mParam->mCalibLeft.fx;   
    mKL.at<float>(1,1) = mParam->mCalibLeft.fy;    
    mKL.at<float>(0,2) = mParam->mCalibLeft.cx;
    mKL.at<float>(1,2) = mParam->mCalibLeft.cy;

    printMatrix("mKL", mKL);

    mKR = cv::Mat::eye(3,3,CV_32F);
    mKR.at<float>(0,0) = mParam->mCalibRight.fx;
    mKR.at<float>(1,1) = mParam->mCalibRight.fy;
    mKR.at<float>(0,2) = mParam->mCalibRight.cx;
    mKR.at<float>(1,2) = mParam->mCalibRight.cy;

    printMatrix("mKR", mKR);

    // DISTORTION COEFF
    mDistCoeffsL = cv::Mat::zeros(4,1,CV_32F);
    mDistCoeffsL.at<float>(0,0) = mParam->mCalibLeft.k1;
    mDistCoeffsL.at<float>(1,0) = mParam->mCalibLeft.k2;
    mDistCoeffsL.at<float>(2,0) = mParam->mCalibLeft.p1;
    mDistCoeffsL.at<float>(3,0) = mParam->mCalibLeft.p2;

    printMatrix("mDistCoeffsL", mDistCoeffsL);

    mDistCoeffsR = cv::Mat::zeros(4,1,CV_32F);
    mDistCoeffsR.at<float>(0,0) = mParam->mCalibRight.k1;
    mDistCoeffsR.at<float>(1,0) = mParam->mCalibRight.k2;
    mDistCoeffsR.at<float>(2,0) = mParam->mCalibRight.p1;
    mDistCoeffsR.at<float>(3,0) = mParam->mCalibRight.p2;   

    printMatrix("mDistCoeffsR", mDistCoeffsR);


    // STEREO CAMERA GEOMETRY
    cv::Mat rotation = cv::Mat::zeros(3,3,CV_32F);
    cv::Mat rotation_tp = cv::Mat::zeros(3,3,CV_32F); 
    cv::Mat translation = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat translation_tp = cv::Mat::zeros(3,1,CV_32F);

    cv::Mat r3X1 = cv::Mat(3,1,CV_32F);
    r3X1.at<float>(0,0) =mParam->mRX;
    r3X1.at<float>(1,0) =mParam->mCV;
    r3X1.at<float>(2,0) =mParam->mRZ;

    cv::Rodrigues(r3X1 /*in*/, rotation /*out*/);  
    rotation_tp=rotation.t();

    translation.at<float>(0,0) =mParam->mBaseline;
    translation_tp = - rotation_tp*translation;

    mHomoLtoR = cv::Mat::eye(3,4,CV_32F);
    mHomoRtoL = cv::Mat::eye(3,4,CV_32F);

    for(uint i=0;i<3;i++)
    {   
        for(uint j=0;j<3;j++)        
            mHomoLtoR.at<float>(i,j) = rotation.at<float>(i,j);        
        mHomoLtoR.at<float>(i,3) = translation.at<float>(i,0);
    }

    for(uint i=0;i<3;i++)
    {   
        for(uint j=0;j<3;j++)        
            mHomoRtoL.at<float>(i,j) = rotation_tp.at<float>(i,j);        
        mHomoRtoL.at<float>(i,3) = translation_tp.at<float>(i,0);
    }

    printMatrix("mHomoLtoR", mHomoLtoR);
    printMatrix("mHomoRtoL", mHomoRtoL);

}

void PointReconstructor::printMatrix(const std::string &caption, const cv::Mat &MAT)
{
    // uint nrow=MAT.rows;
    // uint ncol=MAT.cols;
    // std::cout<< caption << " = " << std::endl;
    // for (uint i=0; i<nrow; i++){
    //     for (uint j=0; j<ncol; j++){
    //         std::cout << MAT.at<float>(i,j) << "  ";
    //     }
    //     std::cout<<std::endl;
    // }    
}

