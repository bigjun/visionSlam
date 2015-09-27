/*
 * feature_matcher.h
 *
 */
#ifndef FEATURE_MATCHER_H_
#define FEATURE_MATCHER_H_

#include <opencv/cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

#include "feature_detection.h"




struct Matches {
    std::vector<cv::KeyPoint> sourceKeyPoints;
    std::vector< Eigen::Vector4f, Eigen::aligned_allocator< Eigen::Vector4f > > sourceLocations;
    cv::Mat soureDescribe;

    std::vector<cv::KeyPoint> targetKeyPoint;
    std::vector< Eigen::Vector4f, Eigen::aligned_allocator< Eigen::Vector4f > > targetLocations;
    cv::Mat targetDescribe;

    std::vector<cv::DMatch> matches;

};


class FeatureMatcher {

public:


    typedef pcl::PointCloud<pcl::PointXYZRGB>  PointCloud;
    typedef PointCloud::Ptr                    PointCloudPtr;
    typedef PointCloud::ConstPtr               PointCloudConstPtr;

    FeatureMatcher();
    FeatureMatcher(cv::Mat& sourceImage, cv::Mat& targetImage, PointCloudPtr& sourceCloud, PointCloudPtr& targetCloud);
    ~FeatureMatcher();

    void setSourceCloud( PointCloudPtr& sourceCloud );
    void setTargetCloud( PointCloudPtr& targetCloud );
    void setSourceImage( cv::Mat& sourceImage );
    void setTargetImage( cv::Mat& targetImage );


    int getMatches( Matches & matches);
    int outlierRemoval (std::vector< cv::DMatch > & rawMatches, std::vector< cv::DMatch >& goodMatches);
    int drawMatches( std::vector<cv::DMatch>& matches, cv::Mat& matchesImage );
    void print3DMatch (std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& source3D, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& target3D);

private:
    std::string matcherType_;
    PointCloudPtr sourceCloudPtr_, targetCloudPtr_;
    cv::Mat sourceImage_, targetImage_;
    std::vector< cv::KeyPoint > sourceKeyPoints_, targetKeyPoints_;
    cv::Mat sourceDescribe_, targetDescribe_;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > sourceFeatureLocation3f_;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > targetFeatureLocation3f_;


    int findMatches( const cv::Mat& source_descriptors, const cv::Mat& target_descriptors, std::vector<cv::DMatch> & matches );




};

#endif
