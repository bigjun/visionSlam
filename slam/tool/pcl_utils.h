/*
 * pcl_utils.h
 *
 */
#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <string>
#include <set>

#include "ParameterReader.h"
#include "pcl_extensions.h"
#include "../registration/registration.h"

using namespace std;
using namespace cv;

namespace utils {

struct CAMERA_INTRINSIC_PARAMETERS {
    double cx, cy;
    double fx, fy;
    double scale;
};


void imagesToPointCloud(  const cv::Mat& depthImg, const cv::Mat& colorImg, const std::string& timeStamp,  pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud, unsigned int downsampling = 1 );

void pointCloudToImage( const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& cloud, cv::Mat& img );
void pointCloudToImages( const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& cloud, cv::Mat& img_rgb, cv::Mat& img_depth );

void projectFeaturesTo3D( std::vector<cv::KeyPoint>& featureLocations2d, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > & featureLocations3d, const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr pointCloud );
void projectFeaturesTo3D (std::vector<cv::KeyPoint>  keyPoints, cv::Mat  depImage, std::vector<DMatch>& matches, std::vector<cv::Point3f> &obj );
void restoreCVMatFromPointCloud( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr cloudIn, cv::Mat& rgbImage);
void restoreCVMatFromPointCloud( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr cloudIn, cv::Mat& rgbImage, cv::Mat& depImage);
PointCloudT::Ptr jointPointCloud( PointCloudT::Ptr orginal, PointCloudT::Ptr target, Eigen::Isometry3d T);


inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera (bool print =false) {

    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    double param;

    pd.get("camera.cx", param);
    camera.cx = param;
    pd.get("camera.cy", param);
    camera.cy = param;
    pd.get("camera.fx", param);
    camera.fx = param;
    pd.get("camera.fy", param);
    camera.fy = param;
    pd.get("camera.scale", param);
    camera.scale = param;

    if (print)
        std::cout << "camera.cx:  " << camera.cx << "  camera.cy:  " << camera.cy<< std::endl
                  <<"camera.fx:  " << camera.fx   << "  camera.fy:  " << camera.fy << std::endl
                  <<"camera.scale:  " << camera.scale << std::endl;




    return camera;






}

}

#endif
