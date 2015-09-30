#include "slamEnd.h"
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <time.h>

using namespace utils;

SlamEnd::SlamEnd() {

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    _globalOptimizer.setAlgorithm( solver );
    _globalOptimizer.setVerbose( false );

    _robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
}
SlamEnd::~SlamEnd() {
    _globalOptimizer.clear();
    _robustKernel->~RobustKernel();
}

int SlamEnd::addVertex( int index, bool setFixed) {

     g2o::VertexSE3* v = new g2o::VertexSE3();
     v->setId( index );
     v->setEstimate( Eigen::Isometry3d::Identity() );
     v->setFixed(setFixed);
     _globalOptimizer.addVertex(v);
     return _globalOptimizer.vertices().size();
 }


 int  SlamEnd::addEdge(int index1, int index2, Eigen::Isometry3d measure, Eigen::Matrix<double, 6, 6> information) {
    //边
     g2o::EdgeSE3* edge = new g2o::EdgeSE3();
     // 连接此边的两个定点
     edge->vertices()[0] = _globalOptimizer.vertex(index1);
     edge->vertices()[1] = _globalOptimizer.vertex(index2);
     //信息矩阵
     edge->setInformation(information);
     //观测值
     edge->setMeasurement(measure);
     edge->setRobustKernel(_robustKernel);
     //把此边加入地图
     _globalOptimizer.addEdge(edge);

 }


 void SlamEnd::checkNearbyLoops(std::vector<FRAME>& frames, FRAME& frame){

     ParameterReader pd;
     int nearby_loops;
     pd.get( "nearby_loops", nearby_loops );

     int beginFrameIdx = 0;
     if (frames.size() <= nearby_loops) {
            beginFrameIdx = 0;
     } else {
            beginFrameIdx = frames.size() - nearby_loops;
     }

     for (int i = 0; i < frames.size(); i++) {
         checkKeyframes(frames[i], frame, true);
     }



 }

 void SlamEnd::checkRandomLoops(std::vector<FRAME>& frames, FRAME& frame) {
     ParameterReader pd;
     int random_loops;
     pd.get( "random_loops", random_loops );
     std::srand( (unsigned int) time(NULL) );



     for (int i = 0; i < random_loops; i++) {
         int index = std::rand()% frames.size();
         checkKeyframes(frames[index], frame, true);
     }

 }

 CHECK_RESULT SlamEnd::checkKeyframes(FRAME& f1, FRAME& f2,  bool is_loops) {
     ParameterReader pd;
     int min_inliers;
     double max_norm, keyframe_threshold, max_norm_lp;
     pd.get( "minimum_inliers",  min_inliers );
     pd.get( "keyframe_threshold", keyframe_threshold);
     pd.get( "max_norm_lp", max_norm_lp );
     pd.get( "max_norm", max_norm );

     int inliers;
     Eigen::Vector7d result = estimateMotion(f1, f2, inliers);
     if (inliers < min_inliers)
         return NOT_MATCHED;

    Eigen::Matrix4d  transform =  Eigen::Matrix4d::Identity();;
    Eigen::Quaterniond q;
    q.x() = result( 3 );
    q.y() = result( 4 );
    q.z() = result( 5 );
    q.w() = result( 6 );
    Eigen::Matrix3d rotate = q.toRotationMatrix();
    transform.block<3,3>(0, 0) = rotate;
    transform.block<3,1>(0, 3) = result.head(3);
    double angle, dist;
    transformToFars(transform, angle, dist);
    double norm = std::abs(angle) + std::abs( dist );

    //计算运动范围是否过大
    if (is_loops == false) {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;
    } else {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }
    //计算运动范围是否过小
    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;

    if (is_loops == false) {
        addVertex(f2.frameID);
    }
    std::cout << "Index 1: "<<f1.frameID<<  "Index 2: "<< f2.frameID << transform << std::endl;
    Eigen::Isometry3d T(transform);
    Eigen::Matrix6d information = Eigen::Matrix6d::Identity();
    for (int i = 0; i < 6; i++)
    {
        information(i, i) = 100;
    }

    addEdge(f1.frameID, f2.frameID, T.inverse(), information);

    return KEYFRAME;

 }


 Eigen::Vector7d SlamEnd::estimateMotion(FRAME& lastFrame, FRAME& currentFrame,  int& inlierNum) {

     std::vector<cv::DMatch> matches;
     FeatureMatcher featMathcher;
     featMathcher.setSourceImage(lastFrame.rgbImg);
     featMathcher.setSourceKeypoints(lastFrame.kp);
     featMathcher.setTargetImage(currentFrame.rgbImg);
     featMathcher.setTargetKeypoints(currentFrame.kp);
     featMathcher.findMatches(lastFrame.desp, currentFrame.desp, matches);
     std::cout << "raw matches size is: " << matches.size() << std::endl;

     std::vector<cv::DMatch> goodMatches;
     featMathcher.outlierRemoval(matches, goodMatches);
     std::cout << "good matches size is: " << goodMatches.size() << std::endl;

     std::vector<cv::Point3f> obj;
     cv::Mat cloneImg;
     cloneImg = lastFrame.depImg.clone();

     projectFeaturesTo3D(lastFrame.kp, cloneImg, goodMatches, obj );
     std::cout << "after project 3d feature is " << goodMatches.size()  << "obj size: " << obj.size()<< std::endl;

     std::vector<cv::Point2f> img;
     for (int i = 0 ; i < goodMatches.size(); i++) {
         int idx = goodMatches[i].trainIdx;
         cv::Point pt = currentFrame.kp[idx].pt;
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

     Eigen::Vector7d pos;
     pos.head(3) = t;
     pos.tail(4) = q.coeffs();

     std::vector<cv::DMatch> inliersMatch;
     for (int i = 0; i < inliers.rows; i++) {
         inliersMatch.push_back(goodMatches[inliers.at<int>(i, 0)]);
     }

     inlierNum = inliers.rows;
     return pos;

 }


/*获取节点的位置*/
Eigen::Isometry3d SlamEnd::getPos(int frameID) {

    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3* > (_globalOptimizer.vertex(frameID));
    Eigen::Isometry3d pos  =vertex->estimate();
    return pos;
}


void SlamEnd::save (std::string saveStr) {
   _globalOptimizer.save(saveStr.c_str());
}


void SlamEnd::optimize(int step) {
    _globalOptimizer.initializeOptimization();
    _globalOptimizer.optimize(step);
}


 void SlamEnd::clear(void) {
     _globalOptimizer.clear();
 }
