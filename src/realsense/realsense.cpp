#include "realsense.h"

namespace td{
    namespace rs{

        void helloz() {

            std::cout << "Hello2, World!" << std::endl;
        }
        //===================================================
        //  PCL_Conversion_rect
        // - Function is utilized to fill a point cloud
        //  object with depth and RGB data from a single
        //  frame captured using the Realsense.
        //===================================================
        cloud_pointer PCL_Conversion_rect(const rs2::depth_frame& depth, const rs2::video_frame &color, const std::vector<cv::Point_<u_int32_t>> &vertex, const float scalaColor2Depth_)
        {
            // Object Declaration (Point Cloud)
            cloud_pointer cloud(new point_cloud);
            if(vertex.size()!=4)
            {
                std::cerr << "need 4 points that can represent a rectangle, from left up and clockwise"  << std::endl;
                return cloud;
            }
            rs2::video_stream_profile depth_profile(depth.get_profile());
            rs2::video_stream_profile color_profile(color.get_profile());
            cv::Mat image(cv::Size(color_profile.width(), color_profile.height()), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
            rs2_intrinsics intr_  = depth_profile.get_intrinsics();
            //================================
            // PCL Cloud Object Configuration
            //================================

            cloud->width  = static_cast<uint32_t>( vertex[1].x-vertex[0].x  );
            cloud->height = static_cast<uint32_t>( vertex[2].y - vertex[1].y );
            cloud->is_dense = false;
            cloud->points.resize( cloud->width * cloud->height );


            // Iterating through all points and setting XYZ coordinates
            // and RGB values
            for (int i = 0; i < cloud->height; i++)
            {
                for (int j = 0; j < cloud->width; ++j) {
                    int index_pcl = i * cloud->width +j;
                    int v = vertex[0].y + i  ;
                    int u = vertex[0].x + j;
                    std::pair<int, int> point2d(u,v);
                    float point3d[3];

                    getXYZfromUV(depth,point2d,point3d,intr_);

                    cloud->points[index_pcl].x = point3d[0];
                    cloud->points[index_pcl].y = point3d[1];
                    cloud->points[index_pcl].z = point3d[2];

                    cloud->points[index_pcl].r = image.at<cv::Vec3b>(scalaColor2Depth_*v,scalaColor2Depth_*u)[2];
                    cloud->points[index_pcl].g = image.at<cv::Vec3b>(scalaColor2Depth_*v,scalaColor2Depth_*u)[1];
                    cloud->points[index_pcl].b = image.at<cv::Vec3b>(scalaColor2Depth_*v,scalaColor2Depth_*u)[0];

                }
            }

            return cloud; // PCL RGB Point Cloud generated
        }
        void getXYZfromUV(const rs2::depth_frame& frame,std::pair<int, int> u, float upoint[3],const rs2_intrinsics intr_)
        {
            float upixel[2]; // From pixel
            // Copy pixels into the arrays (to match rsutil signatures)
            upixel[0] = u.first;
            upixel[1] = u.second;
            // Query the frame for distance
            // Note: this can be optimized
            // It is not recommended to issue an API call for each pixel
            // (since the compiler can't inline these)
            // However, in this example it is not one of the bottlenecks
            auto udist = frame.get_distance(upixel[0], upixel[1]);
            rs2_deproject_pixel_to_point(upoint, &intr_, upixel, udist);

        }
    }//namespace rs
}//namespace td
