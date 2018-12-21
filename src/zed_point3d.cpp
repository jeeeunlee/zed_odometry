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

// Estimate Camera position from reference 3d position and matched 2d points 
void PointReconstructor::solvePnP(const V_POINT3F &keypoints3d, const V_POINT2F &keypoints2d, bool bleft)
{
    int numOfMatches = keypoints3d.size();
    // estimate camera pose
    cv::Mat rvec, tvec;	// rotation & translation vectors  
    
    // set parameters for RANSAC algorithm
    bool useExtrinsicGuess=false;
    int iterationsCount=100;
    float reprojectionError=2.0;
    double confidence=0.99;
    std::vector<uchar> inliners(numOfMatches,0);
    // int flags=cv::SOLVEPNP_ITERATIVE;
    cv::Mat CameraMatrix, DistCoeffs;
    CameraMatrix = bleft ? mKL:mKR;
    DistCoeffs = bleft ? mDistCoeffsL:mDistCoeffsR;

    cv::solvePnPRansac(keypoints3d, keypoints2d, CameraMatrix, DistCoeffs, rvec, tvec, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliners);
    // cv::solvePnP(objectPoints, imagePoints, CameraMatrix, DistCoeffs, rvec, tvec);

    // std::cout<<"rvec: " << rvec.size() << "tvec : " << tvec.size() << std::endl;
    // std::cout<<"rvec: " <<  << "tvec : " <<  << std::endl;
    // save(tvec, "/home/lee/catkin_ws/src/zed_odometry/out/tvec.txt", std::ios::app);
    // save(rvec, "/home/lee/catkin_ws/src/zed_odometry/out/rvec.txt", std::ios::app);

    // check inliners
    V_POINT3F keypoints3dIn;
    V_POINT2F keypoints2dIn;
    int printNum=0;
    for (uint k=0; k<numOfMatches; k++)
    {
        if (inliners[k] > 0.){
            keypoints3dIn.push_back(keypoints3d[k]);   
            keypoints2dIn.push_back(keypoints2d[k]); 
            printNum++;  
        }
    }
    std::cout<< "PnPRansac;Reprojection inliner="<<printNum<<"("<<numOfMatches<<")"<<std::endl;
     

    // extract rotation & translation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat R_inv = R.inv();
    cv::Mat P = -R_inv*tvec;
    double* p = (double *)P.data;

    double* r = (double *)rvec.data;

    // camera position
    printf(":%d:, x=%lf, y=%lf, z=%lf /rx=%lf ry=%lf rz=%lf\n", numOfMatches, p[0], p[1], p[2], r[0], r[1], r[2]);


    cv::solvePnP(keypoints3dIn, keypoints2dIn, CameraMatrix, DistCoeffs, rvec, tvec);
    // save(tvec, "/home/lee/catkin_ws/src/zed_odometry/out/tvec2.txt", std::ios::app);
    // save(rvec, "/home/lee/catkin_ws/src/zed_odometry/out/rvec2.txt", std::ios::app);
    
    // extract rotation & translation matrix    
    cv::Rodrigues(rvec, R);
    R_inv = R.inv();
    P = -R_inv*tvec;
    p = (double *)P.data;
    r = (double *)rvec.data;

    // camera position
    printf(":%d:, x=%lf, y=%lf, z=%lf /rx=%lf ry=%lf rz=%lf\n", numOfMatches, p[0], p[1], p[2], r[0], r[1], r[2]);


}

// Constructor for KeyPoint.
void PointReconstructor::compute(const V_POINT2F &_kpleft, const V_POINT2F &_kpright, V_POINT3F &kp3f)
{
    cv::Mat projMatr1 = cv::Mat::eye(3,4,CV_32F), projMatr2 = cv::Mat::eye(3,4,CV_32F); 
    uint numOfPoints = _kpleft.size();

    // 1. get undistorted 2d point
    // V_POINT2F kpright = UndistortKeyPoints(_kpright, mKR, mDistCoeffsR);
    // V_POINT2F kpleft = UndistortKeyPoints(_kpleft, mKL, mDistCoeffsL);

    // 2. not use undistort
    V_POINT2F kpleft = _kpleft;
    V_POINT2F kpright = _kpright;

    projMatr1 = mKL*projMatr1;
    projMatr2 = mKR*mHomoRtoL; //; 
    
    cv::Mat output(4,numOfPoints,CV_32F);

    cv::triangulatePoints(projMatr1, projMatr2, kpleft, kpright, output);
    cv::Point3f point3f;
    kp3f.clear();
    cv::Mat p3f;
    for(uint i = 0; i < output.cols; i++) 
    {
        cv::Mat _p3h = output.col(i);
        cv::convertPointsFromHomogeneous(_p3h.t(), p3f);
        point3f.x = p3f.at<float>(0);
        point3f.y = p3f.at<float>(1);
        point3f.z = p3f.at<float>(2);
        kp3f.push_back(point3f);
    }

    // save(kpleft, "/home/lee/catkin_ws/src/zed_odometry/out/left.txt");
    // save(kpright, "/home/lee/catkin_ws/src/zed_odometry/out/right.txt");
    // save(kp3f, "/home/lee/catkin_ws/src/zed_odometry/out/p3.txt");
    // save(output, "/home/lee/catkin_ws/src/zed_odometry/out/output.txt");
}


V_POINT2F PointReconstructor::UndistortKeyPoints(const V_POINT2F &mvKeys, const cv::Mat &K, const cv::Mat &distCoeff)
{
    V_POINT2F mvKeysUn;
    cv::Point2f kp;
    int N = mvKeys.size();

    // no distortion -> return original keyPoint
    if(distCoeff.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return mvKeysUn;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i< mvKeys.size(); i++)
    {
        mat.at<float>(i,0)=mvKeys[i].x;
        mat.at<float>(i,1)=mvKeys[i].y;
    }

    // Undistort points
    mat=mat.reshape(2); // make Nx2 1-channel matrix out of Nx1 2-channel
    cv::undistortPoints(mat,mat,K,distCoeff,cv::noArray(), K);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {        
        // mvKeysUn[i].x = K.at<float>(0,0)*mat.at<float>(i,0) + K.at<float>(0,2);
        // mvKeysUn[i].y = K.at<float>(1,1)*mat.at<float>(i,1) + K.at<float>(1,2);
        mvKeysUn[i].x = mat.at<float>(i,0);
        mvKeysUn[i].y = mat.at<float>(i,1);
    }

    // save(mvKeys, "/home/lee/catkin_ws/src/zed_odometry/out/mvKeys.txt");
    // save(mvKeysUn, "/home/lee/catkin_ws/src/zed_odometry/out/mvKeysUn.txt");

    return mvKeysUn;
}

// Constructor for KeyPoint.
void PointReconstructor::reprojection(const V_POINT3F &kp3f, V_POINT2F &reproj_kpleft, V_POINT2F &reproj_kpright)
{
    reproj_kpleft.clear(); reproj_kpright.clear();
    uint numOfPoints = kp3f.size();

    if (numOfPoints<1)
        return;

    cv::Mat ProjectionMat1 = cv::Mat::eye(3,4,CV_32F),  ProjectionMat2 = cv::Mat::eye(3,4,CV_32F);   

    ProjectionMat1 = mKL * ProjectionMat1;
    ProjectionMat2 = mKR * mHomoRtoL;
    
    cv::Mat X3 = cv::Mat(kp3f).reshape(1).t();
    cv::Mat X3bar = cv::Mat(4, numOfPoints, CV_32F,1.0f);

    // std::cout<<"X3 size:"<<X3.size()<<std::endl;
    // std::cout<<"X3bar size:"<<X3bar.size() << ", and X3bar.rowRange(0,3) size: " << X3bar.rowRange(0,3).size()<<std::endl;
    
    (X3).copyTo(X3bar.rowRange(0,3));
    // std::cout << "X3:X3bar"<<X3.at<float>(2,numOfPoints-1)<<" , " <<X3bar.at<float>(2,numOfPoints-1)<< " / "<< X3bar.at<float>(3,numOfPoints-1) << std::endl;

    cv::Mat mat1 = ProjectionMat1 * X3bar; //(ProjectionMat1*Xbar);
    cv::Mat mat2 = ProjectionMat2 * X3bar; //(ProjectionMat1*Xbar);
    cv::Point2f ptemp;
    cv::Mat _p2f;
    for(int i=0; i<numOfPoints; i++)
    {   
        cv::Mat _p2h = mat1.col(i);
        cv::convertPointsFromHomogeneous(_p2h.t(), _p2f);
        ptemp.x = _p2f.at<float>(0);
        ptemp.y = _p2f.at<float>(1);
        reproj_kpleft.push_back(ptemp);

        _p2h = mat2.col(i);
        cv::convertPointsFromHomogeneous(_p2h.t(), _p2f);
        ptemp.x = _p2f.at<float>(0);
        ptemp.y = _p2f.at<float>(1);
        reproj_kpright.push_back(ptemp);
    }


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
    // mDistCoeffsL.at<float>(0,0) = mParam->mCalibLeft.k1;
    // mDistCoeffsL.at<float>(1,0) = mParam->mCalibLeft.k2;
    // mDistCoeffsL.at<float>(2,0) = mParam->mCalibLeft.p1;
    // mDistCoeffsL.at<float>(3,0) = mParam->mCalibLeft.p2;

    // printMatrix("mDistCoeffsL", mDistCoeffsL);

    mDistCoeffsR = cv::Mat::zeros(4,1,CV_32F);
    // mDistCoeffsR.at<float>(0,0) = mParam->mCalibRight.k1;
    // mDistCoeffsR.at<float>(1,0) = mParam->mCalibRight.k2;
    // mDistCoeffsR.at<float>(2,0) = mParam->mCalibRight.p1;
    // mDistCoeffsR.at<float>(3,0) = mParam->mCalibRight.p2;   

    // printMatrix("mDistCoeffsR", mDistCoeffsR);


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
    // cv::Rodrigues(r3X1 /*in*/, rotation_tp /*out*/);  
    // rotation=rotation_tp.t();

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

    // printMatrix("mHomoLtoR", mHomoLtoR);
    // printMatrix("mHomoRtoL", mHomoRtoL);

}

void PointReconstructor::printMatrix(const std::string &caption, const cv::Mat &MAT)
{
    uint nrow=MAT.rows;
    uint ncol=MAT.cols;
    std::cout<< caption << " = " << std::endl;
    for (uint i=0; i<nrow; i++){
        for (uint j=0; j<ncol; j++){
            std::cout << MAT.at<float>(i,j) << "  ";
        }
        std::cout<<std::endl;
    }    
}

void PointReconstructor::save(const V_POINT2F &p,  const char *filename, std::_Ios_Openmode type)
{
    std::ofstream pFile;
    pFile.open(filename, type);
	if (!pFile.is_open())
		std::cout << "Cannot open data File." << std::endl << std::endl;

	for (int i=0;i<p.size();i++)
	{
        pFile << std::setprecision(9) << p[i].x << " " << p[i].y;
		pFile << std::endl;
	}
	pFile.close();   
}

void PointReconstructor::save(const V_POINT3F &p,  const char *filename, std::_Ios_Openmode type)
{
        std::ofstream pFile;
    pFile.open(filename, type);
	if (!pFile.is_open())
		std::cout << "Cannot open data File." << std::endl << std::endl;

	for (int i=0;i<p.size();i++)
	{
        pFile << std::setprecision(9) << p[i].x << " " << p[i].y << " " << p[i].z;
		pFile << std::endl;
	}
	pFile.close();   
}

void PointReconstructor::save(const cv::Mat &p,  const char *filename, std::_Ios_Openmode type)
{
    std::ofstream pFile;
    pFile.open(filename, type);
	if (!pFile.is_open())
		std::cout << "Cannot open data File." << std::endl << std::endl;

	// for (int j=0;j<p.cols;j++)
	// {
    //     for (int i=0;i<p.rows;i++)
    //         pFile << (double) p.at<float>(i,j) << "  ";
	// 	pFile << std::endl;
    // }
	
    double* parray = (double *)p.data;
   	for (int j=0;j<p.cols;j++)
	{
        for (int i=0;i<p.rows;i++)
            pFile <<  parray[j*(p.rows-1)+i]  << "  ";
		pFile << std::endl;
    } 

	pFile.close();   
}
