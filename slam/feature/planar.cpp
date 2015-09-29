#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Eigen>
#include <Eigen/Core>


#include "planar.h"
#include "../tool/ParameterReader.h"
#include "../tool/pcl_utils.h"
#include "../tool/pcl_extensions.h"

#define DEBUG_INFO 1

using namespace std;
using namespace utils;


std::vector <boost::shared_ptr<PLANE> >  Planar::extractPlanes(PointCloudT::Ptr& cloud, cv::Mat& rgb, cv::Mat& dep) {

    std::vector <boost::shared_ptr<PLANE> > planes;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    ParameterReader pd;

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    double distanceThres;
    pd.get("distance_threshold", distanceThres);
    seg.setDistanceThreshold(distanceThres);

    pcl::ExtractIndices<PointT> extract;



    PointCloudT::Ptr tmp(new PointCloudT());
    pcl::copyPointCloud(*cloud, *tmp);

    int i = 0;
    cv::Mat gray;
    cv::cvtColor(rgb, gray,  COLOR_RGB2GRAY);

    double percent;
    pd.get("plane_percent", percent);
    int maxPlanes;
    pd.get("max_planes", maxPlanes);
    CAMERA_INTRINSIC_PARAMETERS cam;
    cam =getDefaultCamera();
    int n = tmp->points.size();

    while (tmp->points.size() > percent*n) {
        seg.setInputCloud(tmp);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            break;
        }

        boost::shared_ptr<PLANE> p( new PLANE() ) ;
        p->coff = *coefficients;


        if (coefficients->values[3] < 0) {
            for (int i = 0; i < 4; i++)
                p->coff.values[i] = -p->coff.values[i];
        }

        PointCloudT::Ptr plane_cloud(new PointCloudT());
        extract.setInputCloud(tmp);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);



        p->image = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));
        p->mask = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));

        int block = 4;
        for (size_t j = 0; j < plane_cloud->points.size(); j++) {

            PointT pt  =plane_cloud->points[j];
            block = int(-1.2*(pt.z) + 10.0);
            block = block > 0? block:0;
            int u = round( pt.x*cam.fx/pt.z + cam.cx);
            int v = round( pt.y*cam.fy/pt.z + cam.cy);



            for ( int k = -block; k < block+1;  k++)
                for (signed int l = -block; l  < block+1;  l++) {
                    if (v+k < 0 || v+k >= 480 || u+l < 0 || u+l >= 640)
                        continue;
                    p->image.ptr(v+k)[u+l]= gray.ptr(v+k)[u+l];
                    p->mask.ptr(v+k) [u+l] = 1;
                }
        }

        cv::imshow("test", p->image);
        cv::waitKey(10);
        cv::equalizeHist(p->image, p->image);

        extract.setNegative(true);
        extract.filter(*tmp);

        i++;
        planes.push_back(p);

        if (i == maxPlanes)
            break;
    }

    return planes;

}


 std::vector <cv::DMatch> Planar::match(std::vector<PlanePtr>& p1, std::vector<PlanePtr> p2) {

     cv::FlannBasedMatcher matcher;
     std::vector<cv::DMatch> matches;
     cv::Mat des1(p1.size(), 4, CV_32F), des2(p2.size(), 4, CV_32F);

     for (size_t i = 0; i < p1.size(); i ++) {
         pcl::ModelCoefficients c = p1[i]->coff;
         float m[1][4] = {c.values[0],c.values[1],c.values[2],c.values[3] };
         cv::Mat mat = cv::Mat(1,4, CV_32F, m);
         mat.row(0).copyTo(des1.row(i));
     }

     for (size_t i = 0; i < p2.size(); i ++) {
         pcl::ModelCoefficients c = p2[i]->coff;
         float m[1][4] = {c.values[0],c.values[1],c.values[2],c.values[3] };
         cv::Mat mat = cv::Mat(1,4, CV_32F, m);
         mat.row(0).copyTo(des2.row(i));
     }

     matcher.match(des1,des2, matches);
     std::cout << "Planes matches" << matches.size() << std::endl;

     return matches;
 }

 std::vector <cv::DMatch> Planar::match(cv::Mat desp1, cv::Mat desp2) {

     //cv::FlannBasedMatcher matcher;
     cv::BFMatcher matcher;
     std::vector<cv::DMatch> matches;

     matcher.match(desp1, desp2, matches);

     double max_dist = 0,  min_dist = 1000;
     for (int i = 0; i < desp1.rows; i++) {
         double dist =  matches[i].distance;
         if (dist < min_dist)
             min_dist = dist;
         if (dist > max_dist)
             max_dist = dist;
     }

     std::vector< cv::DMatch > goodMatches;
     for (size_t i = 0; i < matches.size(); i++) {
         if (matches[i].distance < min_dist *4)
             goodMatches.push_back( matches[i] );
     }
     return goodMatches;
 }


 Eigen::Isometry3d Planar::MatchingPlanar(FRAME&  lastFrame, FRAME& currentFrame) {

     std::vector<PlanePtr> p1 =extractPlanes(lastFrame.cloud, lastFrame.rgbImg, lastFrame.depImg);
     std::vector<PlanePtr> p2 =extractPlanes(currentFrame.cloud, currentFrame.rgbImg, currentFrame.depImg);

     FeatureDection featDection;

     int despSize;
    for (size_t i = 0; i < p1.size(); i++) {
        despSize = featDection.computeKeyPointsAndDesp(p1[i]->image,  p1[i]->kp, p1[i]->desp);
        std::cout << "i: " << i << " despSize:  "  << despSize << std::endl;
        compute3dPosition(p1[i], lastFrame.depImg);
    }

    for (size_t i = 0; i < p2.size(); i++) {
        despSize = featDection.computeKeyPointsAndDesp(p2[i]->image,  p2[i]->kp, p2[i]->desp);
        std::cout << "i: " << i << " despSize:  "  <<despSize << std::endl;
        compute3dPosition(p2[i], currentFrame.depImg);
    }

    std::vector<cv::DMatch> matches = match(p1,p2);
    std::cout << "matches of two planes: " << matches.size() << std::endl;


    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;


    std::vector<cv::KeyPoint> kp1, kp2;
    std::vector<cv::DMatch> match_show;
    int n = 0;
    for (size_t i = 0; i < matches.size(); i++) {

        std::vector<cv::DMatch> kpMatches = pnp(p1[matches[i].queryIdx], p2[matches[i].trainIdx]);
        for(size_t j = 0; j < kpMatches.size(); j++) {
            obj.push_back( p1[matches[i].queryIdx]->kp_pos[kpMatches[j].queryIdx] );
            img.push_back( p2[matches[i].trainIdx]->kp[kpMatches[j].trainIdx].pt );
            kp1.push_back( p1[matches[i].queryIdx]->kp[kpMatches[j].queryIdx] );
            kp2.push_back( p2[matches[i].trainIdx]->kp[kpMatches[j].trainIdx] );

            match_show.push_back( cv::DMatch(n, n, kpMatches[j].distance) );
            n++;

        }

    }

    if (obj.empty()) {
        std::cout << "Object is empty!" << std::endl;
        return Eigen::Isometry3d::Identity();
    }

    CAMERA_INTRINSIC_PARAMETERS cam = getDefaultCamera();
    double camera_matrix[3][3] = { {cam.fx, 0, cam.cx}, {0, cam.fy, cam.cy}, {0, 0, 1}};
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix);
    cv::Mat rvec, tvec;
    cv::Mat inliers;
    double ransacAccuracy;
    ParameterReader pd;
    pd.get("ransac_accuracy", ransacAccuracy);
    cv::solvePnPRansac(obj, img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, ransacAccuracy, 100, inliers);


    std::vector<cv::DMatch> inlierMatches;
    for (int i = 0; i < inliers.rows; i++) {
        inlierMatches.push_back(match_show[ inliers.at<int>(i, 0) ]);
    }

    //For Debug
     if ( DEBUG_INFO ) {
         cv::Mat matchImg;
         cv::drawMatches(lastFrame.rgbImg, kp1, currentFrame.rgbImg, kp2, inlierMatches, matchImg );
         cv::imshow("matches" , matchImg);
         cv::waitKey();
     }

    std::cout << "rvec: " << rvec << "   tvec:  " << tvec << std::endl;

    std::vector< cv::Point3f > obj_new;
    std::vector< cv::Point2f > img_new;
    for (size_t i = 0; i < inlierMatches.size(); i++) {
        obj_new.push_back( obj[inlierMatches[i].queryIdx]);
        img_new.push_back( img[inlierMatches[i].trainIdx]);
    }
    cv::Mat inliers_new;
    cv::solvePnPRansac(obj_new, img_new, cameraMatrix, cv::Mat(), rvec, tvec, true, 100, 3.0, 100, inliers_new);
    std::cout << "rvec: " << rvec << "   tvec:  " << tvec << std::endl;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    cv::Mat R;
    cv::Rodrigues(rvec, R);
   // Eigen::Matrix3d r;
   // cv::cv2eigen(R, r);
    T(0, 0) = R.at<double>(0, 0);
    T(0, 1) = R.at<double>(0, 1);
    T(0, 2) = R.at<double>(0, 2);
    T(1, 0) = R.at<double>(1, 0);
    T(1, 1) = R.at<double>(1, 1);
    T(1, 2) = R.at<double>(1, 2);
    T(2, 0) = R.at<double>(0, 0);
    T(2, 1) = R.at<double>(0, 1);
    T(2, 2) = R.at<double>(0, 2);




    Eigen::Translation<double, 3> trans( tvec.at<double>(0,0), tvec.at<double>(0, 1), tvec.at<double>(0,2) );
    T(0, 3) = trans.x();
    T(1, 3) = trans.y();
    T(2, 3) = trans.z();

   return T.inverse();
 //    return T;

 }


 void Planar::compute3dPosition(PlanePtr& plane, cv::Mat& depImg ) {

     CAMERA_INTRINSIC_PARAMETERS cam = getDefaultCamera();

     for (size_t i = 0; i < plane->kp.size(); i++) {
         cv::KeyPoint kp = plane->kp[i];
         cv::Point3f  pt;
         double u = kp.pt.x, v= kp.pt.y;
         std::cout << " i: "<< i << " u: " << u << "  v: " << v ;
         unsigned short d = depImg.at<unsigned short> (round(u), round(v));
         std::cout << " d: " << d << std::endl;

         std::vector<float> vec =plane->coff.values;
         if (std::isnan( d )  || d == 0) {
             pt.z =1;
            // pt.z = -vec[3] / ( vec[0]*(u-cam.cx)/cam.fx  + vec[1]*(v-cam.cy)/cam.fy + vec[2]) ;
         } else
            pt.z = d/cam.scale;
         pt.x = (u - cam.cx) * pt.z/ cam.fx;
         pt.y = (v - cam.cy) * pt.z/ cam.fy;
         std::cout << " x: " <<  pt.x  << " y: " <<  pt.y<< " z: " <<  pt.z<< std::endl;

         plane->kp_pos.push_back( pt );
     }

 }


 std::vector<cv::DMatch>  Planar::pnp(PlanePtr& p1, PlanePtr& p2) {

    std::vector<cv::DMatch> matches = match(p1->desp, p2->desp);
    if (matches.size() == 0) {
        return std::vector<cv::DMatch> ();
    }

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    for (size_t i = 0; i < matches.size(); i++) {
        obj.push_back( p1->kp_pos[ matches[i].queryIdx ] );
        img.push_back( p2->kp[ matches[i].trainIdx ].pt );
    }

    CAMERA_INTRINSIC_PARAMETERS cam = getDefaultCamera();
    double camera_matrix[3][3] = { {cam.fx, 0, cam.cx}, {0, cam.fy, cam.cy}, {0, 0, 1}};
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix);
    cv::Mat rvec, tvec;
    cv::Mat inliers;
    cv::solvePnPRansac( obj, img, cameraMatrix, cv::Mat(), rvec, tvec, false ,100, 8.0, 100, inliers );

    std::vector< cv::DMatch > inlierMatches;
    for (int i =0; i < inliers.rows; i++) {
        inlierMatches.push_back( matches[ inliers.at<int> (i, 0) ] );
    }

    return inlierMatches;

     return matches;
 }
