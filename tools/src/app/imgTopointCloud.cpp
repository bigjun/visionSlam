#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

#include "utilities/utilities.h"

using namespace tools;
int main(int argv, char** argc) {

    cv::Mat depthImag = cv::imread( "/home/tomson/Data/desk/depth_00001.png", CV_LOAD_IMAGE_ANYDEPTH );
    cv::Mat rgbImage = cv::imread( "/home/tomson/Data/desk/rgb_00001.png", CV_LOAD_IMAGE_ANYCOLOR );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt( new pcl::PointCloud<pcl::PointXYZRGB>() );
    imagesToPointCloud(depthImag,rgbImage, "1", pt);
    pcl::visualization::PCLVisualizer vis("cloud");
    vis.addPointCloud(pt);

    while (1){
        vis.spin();
    }

    return 0;




}
