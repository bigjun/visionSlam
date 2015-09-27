#include <stdio.h>


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;
const std::string CHARTYPE = "SIFT";

double camera_fx = 517.0;
double camera_fy = 517.0;
double camera_cx = 318.6;
double camera_cy = 255.3;
double camera_factor = 5000.0;

int main(int argc, char** argv) {

    if (argc < 3) {
        printf("argc is less than 3\n");
        return -1;
    }


   // initModule_nonfree();

    //读取图像
    cv::Mat img1 = cv::imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat dep1 = cv::imread( argv[2] ,CV_LOAD_IMAGE_ANYDEPTH );
    cv::Mat img2 = cv::imread( argv[3], CV_LOAD_IMAGE_GRAYSCALE );

    /*
    std::vector<KeyPoint> keypoints_surf;
    Ptr<FeatureDetector> surf = FeatureDetector::create("SIFT");
    //surf->create("FAST");
    surf->detect(img1, keypoints_surf);
    cv::Mat img_keypoints;
    cv::drawKeypoints(img1, keypoints_surf,img_keypoints);
    cv::imshow("keypoints", img_keypoints);
    cv::waitKey(0);
    */


    //图像关键点位置
    Ptr<FeatureDetector> detector_img1 = FeatureDetector::create(CHARTYPE);
    std::vector<KeyPoint> keypoints_img1;
    detector_img1->detect(img1, keypoints_img1);

    Ptr<FeatureDetector> detector_img2 = FeatureDetector::create(CHARTYPE);
    std::vector<KeyPoint> keypoints_img2;
    detector_img2->detect(img2, keypoints_img2);

    //图像描述符
    cv::Mat describe1, describe2;
    Ptr<DescriptorExtractor> desExtractor1 = DescriptorExtractor::create(CHARTYPE);
    desExtractor1->compute(img1, keypoints_img1, describe1);
    Ptr<DescriptorExtractor> desExtractor2 = DescriptorExtractor::create(CHARTYPE);
    desExtractor2->compute(img2, keypoints_img2, describe2);


   // detector_img1.compute(img1, keypoints_img1, describe1);
   // detector_img1.compute(img1, keypoints_img1, describe2);

/*
    cv::BFMatcher   matcher;
    std::vector< DMatch > matches;
    matcher.match(describe1, describe2, matches);
    */
    cv::FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( describe1, describe2, matches );
    std::cout << "matches: " << matches.size() << std::endl;

    double  max_dist = 0, min_dist = 1000;
    for (int i=0; i<describe1.rows; i++) {

        double dist = matches[i].distance;
        if (dist < min_dist) {
            min_dist = dist;
        }
        if (dist > max_dist) {
            max_dist = dist;
        }

    }

    std::vector<DMatch> good_matches;
    double match_min_dist = 5.0;
    for (int i = 0; i < describe1.rows; i++) {
        if (matches[i].distance <= std::max(10*min_dist , match_min_dist)) {
            good_matches.push_back(matches[i]);
        }
    }
    std::cout << "good matches: " << good_matches.size() << std::endl;

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    std::vector<cv::KeyPoint>kp1_f, kp2_f;
    std::vector<cv::DMatch> match_f;
    for (size_t i=0; i < good_matches.size(); i++) {

        cv::KeyPoint kp = keypoints_img1[good_matches[i].queryIdx];
        double u = kp.pt.x, v = kp.pt.y;
        unsigned short d = dep1.at<unsigned short>(round(v) ,round(u));

        if (d == 0) {
            continue;
        }
        double z = double(d)/ camera_factor;
        double x = (u - camera_cx)*z/camera_fx;
        double y = (v - camera_cy)*z/camera_cy;
        obj.push_back(cv::Point3f(x, y,z));
        img.push_back(keypoints_img2[good_matches[i].trainIdx].pt);

        match_f.push_back(good_matches[i]);
        kp1_f.push_back(kp);
        kp2_f.push_back(keypoints_img2[good_matches[i].trainIdx]);
        std::cout << "x,y, z: " << x << ", " << y << ", " << z << std::endl;
    }

    double camera_matrix[3][3] ={{camera_fx, 0, camera_cx}, {0, camera_fy, camera_cy}, {0, 0, 1}};
    cv::Mat cameraMatrix(3,3,CV_64F, camera_matrix);
    cv::Mat rvec, tvec;
    cv::Mat inliers;
    cv::solvePnPRansac(obj, img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 9.0, 100, inliers);

    std::vector<DMatch> inliersMatches;
    for (int i=0; i<inliers.rows; i++) {
        inliersMatches.push_back( match_f[inliers.at<int>(i,0)] );
    }
    std::cout << "inliers: " << inliers.rows << std::endl;
    /*
    std::cout << "inliers: " << inliers.rows << std::endl;
    std::ofstream fout( "normal_kp.txt" );
    for (size_t i = 0; i < inliersMatches.size(); i ++ ) {


    }
    */
    cv::Mat image_matches;
   // cv::drawMatches( img1, keypoints_img1, img2, keypoints_img2, inliersMatches, image_matches, Scalar::all(-1), CV_RGB(255, 255, 255), Mat(), 4 );
    cv::drawMatches( img1, keypoints_img1, img2, keypoints_img2, inliersMatches, image_matches);
    cv::imshow( "match", image_matches);









            /*
    Mat img_matches;
    drawMatches(img1, keypoints_img1, img2, keypoints_img2, matches, img_matches);
*/

    //cv::imshow( "Example", img_matches );

    cv::waitKey( 0 );

    return 0;
}


