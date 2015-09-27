#include "parameter_server.h"
#include "ParameterReader.h"

#include "feature_detection.h"
#include "feature_matcher.h"
#include "pcl_utils.h"
#include "stdlib.h"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>


using namespace std;
using namespace utils;

int main(int argc, char** argv) {
    //"minimum_inliers"
    ros::init(argc, argv, "registration");

    std::string rgb1,rgb2, depth1, depth2;
    ParameterReader paramReader;
    paramReader.get( "rgb1", rgb1 );
    paramReader.get( "rgb2", rgb2 );
    paramReader.get( "depth1", depth1 );
    paramReader.get( "depth2", depth2 );


    cv::Mat sourceRgbImage = cv::imread( rgb1, CV_LOAD_IMAGE_ANYCOLOR );
    cv::Mat targetRgbImage = cv::imread( rgb2, CV_LOAD_IMAGE_ANYCOLOR );
    cv::Mat sourceDepthImage = cv::imread( depth1, CV_LOAD_IMAGE_ANYDEPTH );
    cv::Mat targetDepthImage = cv::imread( depth2, CV_LOAD_IMAGE_ANYDEPTH );





    pcl::PointCloud< pcl::PointXYZRGB >::Ptr sourcePointCloud(new pcl::PointCloud< pcl::PointXYZRGB>() );
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud(new pcl::PointCloud< pcl::PointXYZRGB>() );

    imagesToPointCloud( sourceDepthImage, sourceRgbImage, " ",  sourcePointCloud);
    imagesToPointCloud( targetDepthImage, targetRgbImage, " ",  targetPointCloud);



    cv::Mat img1Gray, img2Gray;
    cv::cvtColor( sourceRgbImage, img1Gray, CV_RGB2GRAY );
    cv::cvtColor( targetRgbImage, img2Gray, CV_RGB2GRAY );


    FeatureMatcher featureMatcher;
    featureMatcher.setSourceCloud(sourcePointCloud);
    featureMatcher.setTargetCloud(targetPointCloud);
    featureMatcher.setSourceImage(img1Gray);
    featureMatcher.setTargetImage(img2Gray);



    Matches matches;
    cv::Mat matchesImg;
    featureMatcher.getMatches( matches );
    featureMatcher.drawMatches( matches.matches, matchesImg );


    /*
    FeatureMatcher featureMatcher;
    featur
    */

/*
    FeatureDection featureDection;
    ParameterServer * param = ParameterServer::instance();
    ParameterReader paramReader;
    std::string detector;
    double cx;
    paramReader.get("detector", detector);
    paramReader.get("camera.cx", cx);

    std::cout << "detector: " << detector <<" cx: " <<  cx << std::endl;

    int inlier;
    inlier =  param->get<int>("minimum_inliers");
    //inlier = ParameterServer::instance ()->get<int> ("minimum_inliers");
*/
    /*
    std::vector<KeyPoint> keyPoints;
    cv::Mat descriptor;
    cv::Mat keyPointsImg;
    FeatureDection featureDection;
    cv::Mat img = cv::imread(rgb1.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    featureDection.DetectFeature(img, keyPoints);
    std::cout << "keyPoints size: "  << keyPoints.size() << std::endl;
    featureDection.ExtractFeature(img, keyPoints, descriptor);
    featureDection.drawFeature(img, keyPoints, keyPointsImg);
*/


   // std::cout << "minimum_inliers: " << inlier << std::endl;
    cv::imshow("keyPoints", matchesImg );
    //cv::imshow("keyPoints", img);
    cv::waitKey(-1);
    return 0;

}
