/*
 * feature_detection.h
 *
 */
#ifndef FEATURE_DETECTION_H_
#define FEATURE_DETECTION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <stdlib.h>

#include "../tool/ParameterReader.h"

using namespace cv;
using namespace pcl;

class FeatureDection {
public:
    FeatureDection();
    ~FeatureDection();
    int DetectFeature(cv::Mat& rgbImage, std::vector<cv::KeyPoint>& keyPoints);
    int ExtractFeature(cv::Mat& rgbImage, std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors);
    int computeKeyPointsAndDesp(cv::Mat& rgbImge, std::vector<cv::KeyPoint>& keyPoints, cv::Mat& desc );
    int ProjectPoints3D(std::vector<cv::KeyPoint> keyPoints, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<Eigen::Vector4f> point3d);
    void drawFeature(cv::Mat rgbImage, std::vector<cv::KeyPoint> keyPoints, cv::Mat& keyPointImage);

private:
    std::string featureName_ ;



};

#endif
