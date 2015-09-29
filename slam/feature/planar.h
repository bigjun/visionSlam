/*
*******************planar.h****************************
*/

#ifndef PLANAR_H
#define PLANAR_H

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>

#include <stdlib.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "../tool/pcl_extensions.h"
#include "../registration/registration.h"
#include "feature_detection.h"



struct PLANE {
    pcl::ModelCoefficients coff ; //a,b,c,d
    std::vector<cv::KeyPoint> kp; //keypoints
    std::vector<cv::Point3f> kp_pos;
    cv::Mat desp;
    cv::Mat image;
    cv::Mat mask;
};

typedef boost::shared_ptr<PLANE> PlanePtr;

class Planar {
public:
    std::vector < PlanePtr > extractPlanes(PointCloudT::Ptr& cloud, cv::Mat& rgb, cv::Mat& dep);
    Eigen::Isometry3d MatchingPlanar(FRAME&  lastFrame, FRAME& currentFrame);


private:
    void compute3dPosition(PlanePtr& plane, cv::Mat& depImg );
    std::vector<cv::DMatch>  pnp(PlanePtr& p1, PlanePtr& p2);
    std::vector <cv::DMatch> match(std::vector<PlanePtr>& p1, std::vector<PlanePtr> p2);
    std::vector <cv::DMatch> match(cv::Mat desp1, cv::Mat desp2);

};

#endif //PLANAR_H
