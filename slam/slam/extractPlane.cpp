#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>


#include "stdlib.h"
#include "stdio.h"
#include "iostream"

#include "../tool/pcl_extensions.h"
#include "../tool/pcl_utils.h"

int main(int argc, char** argv) {

    pcl::PCDReader pclRead;
    std::string  pcdFile = "/home/exbot/catkin_ws/dataset/desk/pcd/3.pcd";
    PointCloudT::Ptr cloud( new PointCloudT() );
    pclRead.read(pcdFile, *cloud);


    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.08);

    pcl::ExtractIndices<PointT> extract;

    int n = cloud->points.size();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    int index = 0;
    pcl::PCDWriter pcdWrite;
    while (cloud->points.size() > 0.2*n) {
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            break;
        }

        std::cout << "coefficients : " << coefficients->values[0] << " , "<< coefficients->values[1]
                 << "  ,  " << coefficients->values[2]<<  "  ,  "
                 <<  coefficients->values[3] << std::endl;

        PointCloudT::Ptr plane_cloud(new PointCloudT());
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);


        char buff[128];
        sprintf(buff, "%d.pcd", index );
        pcdWrite.write(buff, *plane_cloud);
        index ++;

        extract.setNegative(true);
        extract.filter(*cloud);


    }


}
