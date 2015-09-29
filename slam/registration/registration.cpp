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

void readFrame(int index, FRAME& frame) {

    std::string datasetPath;
    ParameterReader pd;
    pd.get("datasetpath", datasetPath);

    stringstream ss;
    std::string fileName;
    ss << datasetPath << "/rgb_index/" << index << ".png";
    ss >> fileName;
    frame.rgbImg = cv::imread(fileName, CV_LOAD_IMAGE_ANYCOLOR);

    ss.clear();
    fileName.clear();
    ss << datasetPath << "/dep_index/" << index << ".png";
    ss >> fileName;
    frame.depImg = cv::imread(fileName,CV_LOAD_IMAGE_ANYDEPTH );
    frame.frameID = index;
    frame.cloud = boost::shared_ptr< PointCloudT > ( new PointCloudT() );

}
