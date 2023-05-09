#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "serial.h"
#include "point_cloud_maker.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace D1
{
    class Topic_3D
    {
        public:
            void publishScanImage(ros::Publisher publisher_image_, sensor_msgs::Image::Ptr message_image_,
                                  std::string frame_id_, uint16_t *payload_data_buffer_3d);

            void publishPoint3D(ros::Publisher publisher_point_3d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_3d_, std::string frame_id_,
                                PointCloudMaker &PointCloud, std::vector<uint32_t> color_map, uint16_t *payload_data_buffer_3d);
    };
}


