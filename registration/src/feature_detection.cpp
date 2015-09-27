#include "feature_detection.h"
#include <opencv2/nonfree/nonfree.hpp>
#include <stdlib.h>
using namespace std;
FeatureDection::FeatureDection() {

    ParameterServer* param = ParameterServer::instance();
    featureName_ = param->get<string>("feature_extractor");
    //initModule_nonfree();

    std::cout << "feature_extractor: " << featureName_ << std::endl;

}

FeatureDection::~FeatureDection() {

}

int FeatureDection::DetectFeature( cv::Mat& rgbImage, std::vector<cv::KeyPoint>& keyPoints ) {

    //initModule_nonfree();
    Ptr<FeatureDetector>  detectImage = FeatureDetector::create(featureName_);
    detectImage->detect(rgbImage, keyPoints);

    return 0;
}

int FeatureDection::ExtractFeature( cv::Mat& rgbImage, std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors ) {

    Ptr<DescriptorExtractor> desExtractor = DescriptorExtractor::create(featureName_);
    desExtractor->compute(rgbImage, keyPoints, descriptors);

    return 0;
}

int FeatureDection::ProjectPoints3D(std::vector<cv::KeyPoint> keyPoints, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<Eigen::Vector4f> point3d) {

     return 0;
}

void FeatureDection::drawFeature(cv::Mat rgbImage, std::vector<cv::KeyPoint> keyPoints, cv::Mat& keyPointImage) {
    drawKeypoints(rgbImage,  keyPoints, keyPointImage);
}

