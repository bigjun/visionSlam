#include <stdlib.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>

#include "feature_matcher.h"
#include "ParameterReader.h"
#include "feature_detection.h"
#include "pcl_utils.h"
#include "ransac_transformation.h"
#include "eigen_extensions.h"

using namespace utils;

FeatureMatcher::FeatureMatcher() {
   ParameterReader paramReader;
   paramReader.get("descriptor_matches", matcherType_);
}

FeatureMatcher::~FeatureMatcher() {

}

FeatureMatcher::FeatureMatcher(cv::Mat& sourceImage, cv::Mat& targetImage, PointCloudPtr& sourceCloud, PointCloudPtr& targetCloud) {
    ParameterReader paramReader;
    paramReader.get("descriptor_matches", matcherType_);

    sourceCloudPtr_ = sourceCloud;
    targetCloudPtr_ = targetCloud;
    sourceImage_ = sourceImage;
    targetImage_  = targetImage;


}
void FeatureMatcher::setSourceCloud( PointCloudPtr& sourceCloud ) {

    sourceCloudPtr_ = sourceCloud;
}

void FeatureMatcher::setTargetCloud( PointCloudPtr& targetCloud ) {

    targetCloudPtr_ = targetCloud;
}

void FeatureMatcher::setSourceImage( cv::Mat& sourceImage ) {

    sourceImage_ = sourceImage;
}

void FeatureMatcher::setTargetImage( cv::Mat& targetImage ) {

    targetImage_  = targetImage;
}

int FeatureMatcher::getMatches( Matches &matches ) {


    FeatureDection featureDection;
    featureDection.DetectFeature( sourceImage_,  sourceKeyPoints_ );
    featureDection.DetectFeature( targetImage_,  targetKeyPoints_ );

    projectFeaturesTo3D(sourceKeyPoints_, sourceFeatureLocation3f_, sourceCloudPtr_);
    projectFeaturesTo3D(targetKeyPoints_, targetFeatureLocation3f_, targetCloudPtr_);

    featureDection.ExtractFeature( sourceImage_, sourceKeyPoints_, sourceDescribe_ );
    featureDection.ExtractFeature( targetImage_, targetKeyPoints_, targetDescribe_ );



    std::vector<cv::DMatch> rawMatches, goodMatches;
    this->findMatches( sourceDescribe_, targetDescribe_, rawMatches );
    this->outlierRemoval(rawMatches, goodMatches);

    matches.sourceKeyPoints = sourceKeyPoints_;
    matches.targetKeyPoint = targetKeyPoints_;

    /*
    matches.sourceLocations = ;
    matches.targetLocations = ;
    */

    matches.soureDescribe =  sourceDescribe_;
    matches.targetDescribe = targetDescribe_;
    matches.matches = goodMatches;

    return 0;

}

int FeatureMatcher::findMatches(const cv::Mat& source_descriptors, const cv::Mat& target_descriptors,  std::vector< cv::DMatch >& matches ) {
    if (matcherType_ == "FLANN") {
        cv::FlannBasedMatcher flannMatcher;
        flannMatcher.match(source_descriptors, target_descriptors, matches);
    } else {
        cv::BFMatcher bfMatcher;
        bfMatcher.match(source_descriptors, target_descriptors, matches);
    }

   return matches.size();

}

int FeatureMatcher::outlierRemoval( std::vector<cv::DMatch>& rawMatches, std::vector< cv::DMatch >& goodMatches) {

    ParameterReader paramReader;
    int ransac, minInliers;
    paramReader.get ("RANSAC", ransac);
    paramReader.get ("minimum_inliers", minInliers);

    if (ransac) {

        RansacTransformation ransacTransForm;
        Eigen::Matrix4f resTransForm;
        float rmse =  0.0;

        //print3DMatch (sourceFeatureLocation3f_, targetFeatureLocation3f_);
        ransacTransForm.getRelativeTransformationTo(sourceFeatureLocation3f_, targetFeatureLocation3f_, &rawMatches, resTransForm, rmse, goodMatches, minInliers );
      //  Eigen::Quaterniond q (resTransForm.block <3,3>(0, 0) );
        Eigen::Vector3f tran;
        tran  = resTransForm.block<3, 1>(0, 3);
        /*
        tran (0) = resTransForm(3,0);
        tran (1) = resTransForm(3,1);
        tran (2) = resTransForm(3,2);
        */

        Eigen::Matrix3d rotMax = resTransForm.block<3 ,3 > (0, 0).cast<double> ();
        //Eigen::Matrix3d rotMaxd = rotMaxf.cast<>
        Eigen::Quaterniond q(rotMax);



        std::cout << "T : " << tran(0) << ", " << tran(1) << ", " << tran(2)   << std::endl;
        std::cout << "Q: " << q.w() << ", " << q.x() << ", " << q.y() << ", "<< q.z() << std::endl;
        Eigen::Affine3d aff;



    } else {

        // Outlier detection
        double max_dist = 0;
        double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for (uint i = 0; i < rawMatches.size (); i++)
        {
            double dist = rawMatches[i].distance;
            if (dist < min_dist)
                min_dist = dist;
            if (dist > max_dist)
                max_dist = dist;
        }

        printf ("-- Max dist : %f \n", max_dist);
        printf ("-- Min dist : %f \n", min_dist);

        //-- Find only "good" matches (i.e. whose distance is less than 2*min_dist )
        //-- PS.- radiusMatch can also be used here.
        for (uint i = 0; i < rawMatches.size (); i++)
        {
            if (rawMatches[i].distance < 4 * min_dist)
                goodMatches.push_back (rawMatches[i]);
        }

        for (uint i = 0; i < goodMatches.size (); i++)
        {
            printf ("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i,
                goodMatches[i].queryIdx, goodMatches[i].trainIdx);
        }

    }


    return 0;
}


int FeatureMatcher::drawMatches( std::vector<cv::DMatch>& matches, cv::Mat& matchesImage ) {

    cv::drawMatches(sourceImage_, sourceKeyPoints_, targetImage_, targetKeyPoints_, matches,  matchesImage);
    return 0;
}


void FeatureMatcher::print3DMatch (std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& source3D, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& target3D) {


    for (int i = 0; i < source3D.size(); i++) {
        Eigen::Vector4f srcPt, tarPt;
        srcPt = source3D.at(i);
        tarPt = target3D.at(i);
        std::cout << i << ": source3D: " << srcPt(0) <<", " << srcPt(1) <<", "  << srcPt(2) <<",     target3D:  " << tarPt(0) << ", "<<  tarPt(1) << ", " << tarPt(2) << std::endl;

    }

}
