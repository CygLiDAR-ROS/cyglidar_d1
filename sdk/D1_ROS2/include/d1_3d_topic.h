#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "serial.h"
#include "point_cloud_maker.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace D1
{
    class Topic_3D
    {
        public:
            void publishScanImage(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_, sensor_msgs::msg::Image::SharedPtr message_image_,
                                std::string frame_id_, uint16_t *payload_data_buffer_3d);

            void publishPoint3D(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_3d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud_3d_,
                                std::string frame_id_, PointCloudMaker &PointCloud, std::vector<uint32_t> color_map, uint16_t *distance_buffer_3d_);
    };
}
