#pragma once

#ifndef ZED_COMMON_H
#define ZED_COMMON_H

#include <vector>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::KeyPoint> V_KEYPOINTS;
typedef std::vector<cv::Point2f>  V_POINT2F;
typedef std::vector<cv::Point3f>  V_POINT3F;
typedef std::vector<cv::DMatch> V_MATCHES;

typedef cv::Mat IMAGE;
typedef cv::Mat DESCRIPTORS;

#endif