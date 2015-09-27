#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "../tool/ParameterReader.h"
#include "../registration/registration.h"
#include "../tool/pcl_utils.h"
#include "../feature/feature_detection.h"
#include  "../feature/feature_matcher.h"
#include "../tool/eigen_extensions.h"

// -DCMAKE_BUILD_TYPE=Debug

using namespace std;
using namespace utils;

Eigen::Vector7d Maching(boost::shared_ptr<FRAME> lastFrame, boost::shared_ptr<FRAME>  currentFrame) ;

void  usage() {
    std::cout << "Usage: start_index end_index" << std::endl;
}

int main(int argc, char** argv)
{
    /*
    if (argc != 3) {
        usage();
        return -1;
    }


    int startIndex = std::atoi(argv[1]);
    int endIndex = std::atoi(argv[2]);
    */
    int startIndex = 1;
    int endIndex = 200;

    ofstream fout("/home/exbot/catkin_ws/dataset/xyz/odometry.txt",  ios::trunc);

    boost::shared_ptr<FRAME> lastFrame;
    boost::shared_ptr<FRAME> currentFrame;

    FeatureDection featDection;
    lastFrame = readFrame(startIndex);
    imagesToPointCloud(lastFrame->depImg, lastFrame->rgbImg, " ", lastFrame->cloud);
    int kpSize = featDection.computeKeyPointsAndDesp(lastFrame->rgbImg, lastFrame->kp, lastFrame->desp);
    if (kpSize < 10) {

        std::cout << "the frame id %d  key points size is : %d, too little!\n" << startIndex << kpSize;
        return -1;
    }

    for (int index = startIndex +3; index < endIndex;  index = index+3) {

        currentFrame = readFrame (index);
        imagesToPointCloud(currentFrame->depImg, currentFrame->rgbImg, " ", currentFrame->cloud);
        int kpSize = featDection.computeKeyPointsAndDesp(currentFrame ->rgbImg, currentFrame ->kp, currentFrame ->desp);
        if (kpSize < 10) {

            std::cout << "the frame id %d  key points size is : %d, too little!\n" << startIndex << kpSize;
            return -1;
        }


        //to do the computer
        Eigen::Vector7d  pos;
        pos = Maching(lastFrame, currentFrame);

        fout << lastFrame->frameID <<"\t"<<currentFrame->frameID
             << "\t"<< pos(0)<< "\t"<< pos(1)<< "\t"<< pos(2)<< "\t"<< pos(3)
             << "\t"<< pos(4)<< "\t"<< pos(5)<< "\t"<< pos(6) << std::endl;





        lastFrame =currentFrame;
      //  cv::imshow("rgb", lastFrame->rgbImg);
    //    cv::waitKey(1000);

    }

    fout.close();

    return 0;

}


Eigen::Vector7d Maching(boost::shared_ptr<FRAME> lastFrame, boost::shared_ptr<FRAME>  currentFrame) {

    std::vector<cv::DMatch> matches;
    FeatureMatcher featMathcher;
    featMathcher.setSourceImage(lastFrame->rgbImg);
    featMathcher.setSourceKeypoints(lastFrame->kp);
    featMathcher.setTargetImage(currentFrame->rgbImg);
    featMathcher.setTargetKeypoints(currentFrame->kp);
    featMathcher.findMatches(lastFrame->desp, currentFrame->desp, matches);
    std::cout << "raw matches size is: " << matches.size() << std::endl;

    std::vector<cv::DMatch> goodMatches;
    featMathcher.outlierRemoval(matches, goodMatches);
    std::cout << "good matches size is: " << goodMatches.size() << std::endl;

    std::vector<cv::Point3f> obj;
    projectFeaturesTo3D(lastFrame->kp, lastFrame->depImg, goodMatches, obj );
    std::cout << "after project 3d feature is " << goodMatches.size()  << "obj size: " << obj.size()<< std::endl;

    std::vector<cv::Point2f> img;
    for (int i = 0 ; i < goodMatches.size(); i++) {
        int idx = goodMatches[i].trainIdx;
        cv::Point pt = currentFrame->kp[idx].pt;
        img.push_back(pt);
    }


    CAMERA_INTRINSIC_PARAMETERS cam;
    cam = getDefaultCamera();
    double camMatrix[3][3] = { { cam.fx, 0, cam.cx }, { 0, cam.fy, cam.cy }, { 0, 0, 1 }};
    cv::Mat cameraMatrix(3, 3, CV_64F, camMatrix );
    cv::Mat rvec, tvec;
    cv::Mat inliers;
    cv::solvePnPRansac(obj, img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 8.0, 100, inliers);

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
    Eigen::Quaterniond q(r);
    Eigen::Vector3d t;
    cv::cv2eigen(tvec,t );
    //t = -t;
   // q =  q.inverse();
   // std::cout << "t: " << t(0) << ", " << t(1) << ", "<< t(2)<< std::endl;
   // std::cout << "q: " << q.w() << ", " << q.x() << ", "<< q.y()<< ", "<< q.z() << std::endl;

    Eigen::Vector7d pos;
    pos.head(3) = t;
    pos.tail(4) = q.coeffs();

    std::vector<cv::DMatch> inliersMatch;
    for (int i = 0; i < inliers.rows; i++) {
        inliersMatch.push_back(goodMatches[inliers.at<int>(i, 0)]);
    }
    std::cout << "inlierMatches size is: "<< inliersMatch.size() << std::endl;

    cv::Mat matchImag;
    featMathcher.drawMatches(inliersMatch, matchImag);
   // cv::imshow("matches", matchImag);
   // cv::waitKey(100);

   // Eigen::Isometry3d T;
    return pos;

}
