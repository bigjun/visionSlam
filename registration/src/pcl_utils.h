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

using namespace std;
using namespace cv;

namespace utils {

    void imagesToPointCloud( const cv::Mat& depthImg, const cv::Mat& colorImg, const std::string& timeStamp, pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud, unsigned int downsampling = 1 );

    void pointCloudToImage( const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& cloud, cv::Mat& img );
    void pointCloudToImages( const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& cloud, cv::Mat& img_rgb, cv::Mat& img_depth );

    void projectFeaturesTo3D( std::vector<cv::KeyPoint>& featureLocations2d, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > & featureLocations3d, const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr pointCloud );
    void restoreCVMatFromPointCloud( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr cloudIn, cv::Mat& rgbImage);
    void restoreCVMatFromPointCloud( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr cloudIn, cv::Mat& rgbImage, cv::Mat& depImage);

}

#endif
