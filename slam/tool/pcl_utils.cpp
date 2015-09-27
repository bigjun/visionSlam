#include "pcl_utils.h"
#include "ParameterReader.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace cv;

void utils::imagesToPointCloud( const cv::Mat& depthImg, const cv::Mat& colorImg, const std::string& timeStamp, pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud, unsigned int downsampling ) {

    ParameterReader paramReader;
    double focalLengthx,  focalLengthy, cx, cy, scale;
    paramReader.get("camera.fx", focalLengthx);
    paramReader.get("camera.fy", focalLengthy);
    paramReader.get("camera.cx", cx);
    paramReader.get("camera.cy", cy);
    paramReader.get("camera.scale", scale);



    cloud->header.frame_id = "openni_rgb_optical_frame";
    cloud->is_dense = true;
    cloud->height = depthImg.rows / downsampling;
    cloud->width = depthImg.cols / downsampling;
    cloud->sensor_origin_ = Eigen::Vector4f( 0.f, 0.f, 0.f, 1.f );
    cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
    cloud->points.resize( cloud->height*cloud->width );

    const float invfocalLengthx = 1.f / focalLengthx;
    const float invfocalLengthy = 1.f / focalLengthy;
    const float centerX = cx;
    const float centerY = cy;
    const float factor = 1.f / scale;

    const unsigned short* depthdata = reinterpret_cast<const unsigned short*>( &depthImg.data[0] );
    const unsigned char* colordata = &colorImg.data[0];
    int idx = 0;
    for( unsigned int y = 0; y < depthImg.rows; y++ ) {
        for( unsigned int x = 0; x < depthImg.cols; x++ ) {

            if( x % downsampling != 0 || y % downsampling != 0 ) {
                colordata += 3;
                depthdata++;
                continue;
            }

            pcl::PointXYZRGB& p = cloud->points[idx];

            if( *depthdata == 0 ) { //|| factor * (float)(*depthdata) > 10.f ) {

               p.x = std::numeric_limits<float>::quiet_NaN();
               p.y = std::numeric_limits<float>::quiet_NaN();
               p.z = std::numeric_limits<float>::quiet_NaN();



            }
            else {
                float xf = x;
                float yf = y;
                float dist = factor * (float)(*depthdata);
                p.x = (xf-centerX) * dist * invfocalLengthx;
                p.y = (yf-centerY) * dist * invfocalLengthy;
                p.z = dist;
            }

            depthdata++;

            int b = (*colordata++);
            int g = (*colordata++);
            int r = (*colordata++);

            int rgb = ( r << 16 ) + ( g << 8 ) + b;
            p.rgb = * ( reinterpret_cast< float* > ( &rgb ) );

            idx++;


        }
    }
}


void utils::projectFeaturesTo3D (std::vector<cv::KeyPoint>& featureLocations2d, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > & featureLocations3d, const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr pointCloud) {

    int index = -1;
    for (unsigned int i = 0; i < featureLocations2d.size (); /*increment at end of loop*/)
    {
      ++index;

      cv::Point2f p2d = featureLocations2d[i].pt;
      pcl::PointXYZRGB p3d = pointCloud->at ((int) p2d.x, (int) p2d.y);

      // Check for invalid measurements
      if (isnan (p3d.x) || isnan (p3d.y) || isnan (p3d.z))
      {
        ROS_DEBUG ("Feature %d has been extracted at NaN depth. Omitting", i);
        featureLocations2d.erase (featureLocations2d.begin () + i);
        continue;
      }

      featureLocations3d.push_back (Eigen::Vector4f (p3d.x, p3d.y, p3d.z, 1.0));
      //featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
      i++; //Only increment if no element is removed from vector
    }
}

void utils::projectFeaturesTo3D (std::vector<cv::KeyPoint> keyPoints, cv::Mat depImage, std::vector<DMatch>& matches, std::vector<cv::Point3f>& obj ) {

    CAMERA_INTRINSIC_PARAMETERS cam ;
    cam = getDefaultCamera();
    //std::cout << "cam:" << cam.cx << "  "<< cam.cy << " " << cam.fx << "  " << cam.fy <<  " " << cam.scale << std::endl;

    std::vector<cv::DMatch> goodMatches;
   // const unsigned short* depthdata = reinterpret_cast<const unsigned short*>( &depImage.data[0] );


    for (int i = 0; i < matches.size(); i++) {
        std::cout << "kp size: " << keyPoints.size() << "  index:" << matches[i].queryIdx << std::endl;
        cv::KeyPoint kp = keyPoints [matches[i].queryIdx];

        double u = kp.pt.x;
        double v = kp.pt.y;
        std::cout << " u: " << u << " v: " << v  << std::endl;

      //  if (std::isnan(depImage.at<unsigned short>(round(u), round(v))))
       // int ui = round(u);
       // int vi = round(v);
       //     continue;
      //  unsigned short d = (unsigned short) *(depthdata +ui*640 + vi);

        unsigned short d = depImage.at<unsigned short>(round(u), round(v));

        if (std::abs(d)  <  0.001  ||   u <0 || u >= 480 || v < 0 || v >= 480   ) {
            continue;
        }


        double z = double(d) / cam.scale;
        double x = (u - cam.cx) * z / cam.fx;
        double y = (v - cam.cy) * z / cam.fy;
        obj.push_back(cv::Point3f(x, y, z));
        goodMatches.push_back(matches[i]);
        std::cout << "                                   x: " << x << " y: " << y << " z: " << z << std::endl;
    }

    matches.clear();
    for (int i = 0; i < goodMatches.size();  i++) {
        matches.push_back(goodMatches[i]);
    }

}


void utils::restoreCVMatFromPointCloud (pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr cloudIn, cv::Mat& rgbImage) {

    rgbImage.create(cloudIn->height, cloudIn->width, CV_8UC3);
    for (uint rows = 0; rows < cloudIn->height; rows++)
    {
      for (uint cols = 0; cols < cloudIn->width; ++cols)
      {
        //      restored_image.at<uint8_t>(rows, cols) = cloud_in->at(cols, rows).r;
        rgbImage.at<cv::Vec3b> (rows, cols)[0] = cloudIn->at (cols, rows).b;
        rgbImage.at<cv::Vec3b> (rows, cols)[1] = cloudIn->at (cols, rows).g;
        rgbImage.at<cv::Vec3b> (rows, cols)[2] = cloudIn->at (cols, rows).r;
      }
    }
}


PointCloudT::Ptr utils::jointPointCloud( PointCloudT::Ptr orginal, PointCloudT::Ptr target, Eigen::Isometry3d T) {

    PointCloudT::Ptr output( new PointCloudT() );
     PointCloudT::Ptr tmp( new PointCloudT() );
    pcl::transformPointCloud( *orginal, *output, T.matrix() );
    *output +=  *target;

    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize( 0.01, 0.01, 0.01);
    voxel.setInputCloud(output);
    voxel.filter(*tmp);

    //return tmp;

    return output;


}
