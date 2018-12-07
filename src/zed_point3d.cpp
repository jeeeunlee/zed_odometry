#include "zed_point3d.h"

bool PointReconstructor::mbInitialzedGetInformation=true;

PointReconstructor::PointReconstructor()
{}

PointReconstructor::PointReconstructor(const std::string &source)
{
    std::cout<<"PointReconstructor compute"<<std::endl;

    if(mbInitialzedGetInformation)
    {
        std::cout<<"POINT3D mbInitialzedGetInformation"<<std::endl;

        mParam = ZParam::getInstance(source);   
        std::cout<<"POINT3D getInstance"<<std::endl;         
        getCalibInfo(mParam);
        std::cout<<"POINT3D getCalibInfo"<<std::endl;
        mbInitialzedGetInformation=false;
    }

}

// Constructor for KeyPoint.
Point3D PointReconstructor::compute(const cv::KeyPoint &kpleft, const cv::KeyPoint &kpright)
{
    compute(kpleft.pt, kpright.pt);
}

// Constructor for Point2f.
Point3D PointReconstructor::compute(const cv::Point2f &p2left, const cv::Point2f &p2right)
{
    // This is done only for the first Frame (or after a change in the calibration)
    Point3D point;



    point.X = 0.;
    point.Y = 0.;
    point.Z = 0.;

    return point;
}

void PointReconstructor::getCalibInfo(const ZParam *param){


    cv::Mat Kl = cv::Mat::eye(3,3,CV_32F);    
    Kl.at<float>(0,0) = param->mCalibLeft.fx;    
    Kl.at<float>(1,1) = mParam->mCalibLeft.fy;     
    Kl.at<float>(0,2) = mParam->mCalibLeft.cx;
    Kl.at<float>(1,2) = mParam->mCalibLeft.cy;

    cv::Mat Kr = cv::Mat::eye(3,3,CV_32F);
    Kr.at<float>(0,0) = mParam->mCalibLeft.fx;
    Kr.at<float>(1,1) = mParam->mCalibLeft.fy;
    Kr.at<float>(0,2) = mParam->mCalibLeft.cx;
    Kr.at<float>(1,2) = mParam->mCalibLeft.cy;



    cv::Mat rotation = cv::Mat(3,3,CV_32F);
    cv::Mat rotation_tp = cv::Mat(rotation.cols, rotation.rows, CV_32F); 


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

    cv::Mat HomoLtoR = cv::Mat::eye(3,4,CV_32F);
    cv::Mat Rot = HomoLtoR.colRange(0,2).rowRange(0,2);
    cv::Mat pos = HomoLtoR.colRange(0,2).rowRange(3,3);

    rotation.copyTo(Rot);
    translation.copyTo(pos);
    
    cv::Mat HomoRtoL = cv::Mat::eye(3,4,CV_32F);
    Rot = HomoRtoL.colRange(0,2).rowRange(0,2);
    pos = HomoRtoL.colRange(0,2).rowRange(3,3);

    rotation_tp.copyTo(Rot);
    translation_tp.copyTo(pos);


    

    std::cout<<"translation";    
    for(uint i=0;i<3;i++)
    {              std::cout<<", "<< translation.at<float>(i,0);    }
    std::cout<<std::endl;

        std::cout<<"translation_tp";    
    for(uint i=0;i<3;i++)
    {              std::cout<<", "<< translation_tp.at<float>(i,0);    }
    std::cout<<std::endl;

    std::cout<<"rotation";    
    for(uint i=0;i<3;i++)
    {   
        for(uint j=0;j<3;j++)
        {
            std::cout<<", "<< rotation.at<float>(i,j);
        }
        std::cout<<std::endl;
    }


    std::cout<<"rotation_tp";    
    for(uint i=0;i<3;i++)
    {   
        for(uint j=0;j<3;j++)
        {
            std::cout<<", "<< rotation_tp.at<float>(i,j);
        }
        std::cout<<std::endl;
    }



    std::cout<<"HomoLtoR";    
    for(uint i=0;i<3;i++)
    {   
        for(uint j=0;j<4;j++)
        {
            std::cout<<", "<< HomoLtoR.at<float>(i,j);
        }
        std::cout<<std::endl;
    }

    std::cout<<"HomoRtoL";    
    for(uint i=0;i<3;i++)
    {   
        for(uint j=0;j<4;j++)
        {
            std::cout<<", "<< HomoRtoL.at<float>(i,j);
        }
        std::cout<<std::endl;
    }
    // cv::Rodrigues([Rx, Ry, Rz] /*in*/, RotMatrix3x3 /*out*/)

}

