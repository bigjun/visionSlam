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
#include "../tool/eigen_extensions.h"
#include "../feature/planar.h"

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
    int startIndex = 5;
    int endIndex = 15;

    ofstream fout("/home/exbot/catkin_ws/dataset/desk/odometry.txt",  ios::trunc);

    boost::shared_ptr<FRAME> lastFrame;
    boost::shared_ptr<FRAME> currentFrame;

    lastFrame = readFrame(startIndex);
    utils::imagesToPointCloud(lastFrame->depImg, lastFrame->rgbImg, " ", lastFrame->cloud);
    Planar planar;

    PointCloudT::Ptr globalCloud( new PointCloudT() );
    pcl::copyPointCloud(*lastFrame->cloud, *globalCloud);

    pcl::PCDWriter pcdWrite;
    char buff[128];

    for (int index = startIndex +2; index < endIndex;  index = index +3) {

        currentFrame = readFrame (index);
        utils::imagesToPointCloud(currentFrame->depImg, currentFrame->rgbImg, " ", currentFrame->cloud);

        Eigen::Isometry3d T =  planar.MatchingPlanar(lastFrame, currentFrame);
        //Eigen::matrx
        Eigen::Matrix3d R = T.rotation();
        Eigen::Quaterniond q(R );
        Eigen::Vector3d t = T.translation();



        //to do the computer
       std::cout << lastFrame->frameID <<"\t"<<currentFrame->frameID <<  "\t" <<"q: " << q.x() << " " << q.y()
                 << " " << q.z() << " " <<q.w() << "  T:" << t(0) << " " << t(1) << " " << t(2) << std::endl;

        globalCloud = utils::jointPointCloud( currentFrame->cloud, globalCloud, T );
        sprintf(buff, "output%d.pcd", index);
        std::string str = buff;
        pcdWrite.write(str, *globalCloud);
       // pcdWrite.write(str, *currentFrame->cloud);





        lastFrame =currentFrame;
      //  cv::imshow("rgb", globalCloud);
     //   cv::waitKey(1000);

    }

    fout.close();

    return 0;

}


