#include <sstream>
#include <fstream>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include "registration.h"
#include "../tool/ParameterReader.h"
#include "../tool/pcl_extensions.h"


using namespace std;
using namespace cv;

boost::shared_ptr<FRAME>  readFrame(int index) {

   boost::shared_ptr<FRAME>  f(new FRAME()  );
   //FRAME* f(new FRAME () );

    std::string datasetPath;
    ParameterReader pd;
    pd.get("datasetpath", datasetPath);

    stringstream ss;
    std::string fileName;
    ss << datasetPath << "/rgb_index/" << index << ".png";
    ss >> fileName;
    f->rgbImg = cv::imread(fileName, CV_LOAD_IMAGE_ANYCOLOR);

    ss.clear();
    fileName.clear();
    ss << datasetPath << "/dep_index/" << index << ".png";
    ss >> fileName;
    f->depImg = cv::imread(fileName,CV_LOAD_IMAGE_ANYDEPTH );
    //f->depImg.convertTo(f->depImg, CV_32F);
    f->frameID = index;
    //PointCloudT::Ptr pt(new PointCloudT() );

    f->cloud = boost::shared_ptr< PointCloudT > ( new PointCloudT() );
   // cv::imshow("debug", f->depImg);
 //   cv::waitKey(0);
    //std::cout << f->depImg << std::endl;
    /*
    const unsigned short* depthdata = reinterpret_cast<const unsigned short*>( &f->depImg.data[0] );

    for (int i = 0; i < 640; i++)
        for (int j = 0;j< 480; j++)
        {
           //unsigned short d = f->depImg.at< unsigned short>(i, j);
            unsigned short d = (unsigned short)*(depthdata + i* 480 + j);
           std::cout << "d:   " << d << " i:  " << i << " j:" << j << "depth: " << f->depImg.depth() <<std::endl;
        }
        */
    //Only for Debug


    return f;
}
