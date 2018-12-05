#include <iostream>
#include <fstream>
#include <iomanip> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);


int main ( int argc, char** argv ) // argv[1]=calibration yaml, argv[2]=image file path
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);

    int nImages = min(vstrImageLeft.size(), vstrImageRight.size());

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }
    else
    {
        cout<<"nImages: "<< nImages <<endl;
        cout<<"ex: vstrImageLeft[0]= "<< vstrImageLeft[0]<< endl;
    }

    // Main loop
    cv::Mat imLeft, imRight;
    std::vector<KeyPoint> keypoints_left, keypoints_right;
    Mat descriptors_left, descriptors_right;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> matches, good_matches;
    
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        /*  CV_LOAD_IMAGE_COLOR : 이미지 파일을 Color로 읽어들입니다. 투명한 부분은 무시되며, Default값입니다.
            CV_LOAD_IMAGE_GRAYSCALE : 이미지를 Grayscale로 읽어 들입니다. 실제 이미지 처리시 중간단계로 많이 사용합니다.
            CV_LOAD_IMAGE_UNCHANGED : 이미지파일을 alpha channel까지 포함하여 읽어 들입니다.    */        
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_COLOR); 
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_COLOR);

        detector->detect ( imLeft, keypoints_left );
        detector->detect ( imRight, keypoints_right );
        
        descriptor->compute ( imLeft, keypoints_left, descriptors_left );
        descriptor->compute ( imRight, keypoints_right, descriptors_right );

        // display feature
        //static Mat outimg_left;
        //drawKeypoints( imLeft, keypoints_left, outimg_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        //imshow("ORB_left",outimg_left);

        // extract match
        matcher->match ( descriptors_left, descriptors_right, matches );
        
        // extract good match using NNDR(Nearest Neighbour Distance Ratio) 
        double ratio = 0.6;
        std::vector< vector<DMatch > > nnMatches;
        std::vector< DMatch > good_NNmatches;
        matcher->knnMatch(descriptors_left, descriptors_right, nnMatches, 2);
        for(int k = 0; k < nnMatches.size(); k++)
        {
            if(nnMatches[k][0].distance / nnMatches[k][1].distance < ratio)
            {
                good_NNmatches.push_back(nnMatches[k][0]);
            }
        }

        cout<< "# of good Matches: " << good_NNmatches.size() << " / Matches: " << matches.size() << endl;


    }   
    
    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/left";
    string strPrefixRight = strPathToSequence + "/right";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i+1;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
