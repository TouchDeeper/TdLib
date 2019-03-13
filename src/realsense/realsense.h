//
// Created by wang on 19-3-9.
//

#ifndef TDLIBRARY_REALSENSE_H
#define TDLIBRARY_REALSENSE_H
//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <librealsense2/rsutil.h>
#include <iostream>
#include <vector>
//PCL
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>   // Include OpenCV API
namespace td{
    namespace rs{
//        typedef pcl::PointXYZRGB RGB_Cloud;
//        typedef pcl::PointCloud<RGB_Cloud> point_cloud;
//        typedef point_cloud::Ptr cloud_pointer;
        void helloz();
//        cloud_pointer PCL_Conversion_rect(const rs2::depth_frame& depth, const rs2::video_frame &color, const std::vector<cv::Point_<u_int32_t>> &vertex,const rs2_intrinsics intr_);
//        void getXYZfromUV(const rs2::depth_frame& frame,std::pair<int, int> u, float upoint[3],const rs2_intrinsics intr_);

    }//namespace rs
}//namespace td

#endif //TDLIBRARY_REALSENSE_H