/*************************************************************************
	> File Name: registration.h

 ************************************************************************/
#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../tool/pcl_utils.h"
#include "../tool/pcl_extensions.h"

struct FRAME {
	int frameID;
	cv::Mat rgbImg, depImg;
    cv::Mat desp;
    PointCloudT::Ptr  cloud;
	std::vector<cv::KeyPoint> kp;
};

typedef boost::shared_ptr<FRAME>  FramePtr;

void readFrame(int index, FRAME& frame);
#endif
