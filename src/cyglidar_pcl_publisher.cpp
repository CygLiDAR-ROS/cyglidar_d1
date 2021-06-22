#include <cyglidar_pcl.h>

const float RADIAN = MATH_PI / HALF_ANGLE;
const float DEGREE = HALF_ANGLE / MATH_PI;

int DATABUFFER_SIZE_2D, DATABUFFER_SIZE_3D, DATASET_SIZE_2D, DATASET_SIZE_3D;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointXYZRGBA;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D;
sensor_msgs::LaserScan::Ptr scan_laser;
tf::TransformListener *tf_listener;

ros::Publisher pub_2D, pub_3D, pub_scan;

ros::Time current_time_laser, last_time_laser;
double time_diff_laser;

uint8_t *cloudBuffer;
float *cloudSetBuffer;
uint8_t *cloudBuffer_2D;
float *cloudSetBuffer_2D;

uint8_t* bufferPtr_2D;
uint8_t* bufferPtr01;
uint8_t* bufferPtr02;
uint8_t* bufferPtr;

uint8_t FIRST, SECOND, THIRD, LSB, MSB;

float centerX_3D, centerY_3D, width, height;
float actualX = 0.0, actualY = 0.0;

float angleRadian = RADIAN * -RIGHT_ANGLE;

float data1, data2, actualDistance, length;
uint8_t currentBuffer;

static std::vector<uint32_t> colorArray;
uint32_t rgb_setup;
uint8_t r_setup, g_setup, b_setup;
void colorBuffer()
{
    int colors = 0;
    r_setup = 254;
    g_setup = 0;
    b_setup = 0;

    for (int colorSet = 0; colorSet < 2; colorSet++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int colorCnt = 0; colorCnt < COLOR_MAX; colorCnt++)
            {
                switch (colors)
                {
                    // RED -> YELLOW
                    case 0:
                        g_setup++;
                        break;
                    // YELLOW -> BLUE
                    case 1:
                        r_setup--;
                        g_setup--;
                        b_setup++;
                        break;
                    // BLUE -> RED
                    case 2:
                        r_setup++;
                        b_setup--;
                        break;
                }

                rgb_setup = ((uint32_t)r_setup << 16 | (uint32_t)g_setup << 8 | (uint32_t)b_setup);
                colorArray.push_back(rgb_setup);

                if (colorCnt == COLOR_MAX - 1)
                {
                    colors++;
                }
            }
        }
    }

    //ROS_INFO("COLORS STORED (%d)", colorArray.size());
}

bool drawing = false;
int distanceCnt_2D;
float angleStep_2D, point_angle_var_2D, point_angle_2D;
float tempX_2D, tempY_2D;
uint8_t cloudScatter_2D()
{
    if (!drawing)
    {
        drawing = true;
        memcpy(cloudBuffer_2D, &bufferPtr_2D[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_2D);

        distanceCnt_2D = 0;

        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_2D - PAYLOAD_SIZE) - 1; dataLength+=2)
        {
            MSB = cloudBuffer_2D[dataLength];
            LSB = cloudBuffer_2D[dataLength + 1];

            data1 = (float)((MSB << 8) | LSB);

            cloudSetBuffer_2D[distanceCnt_2D++] = data1;
        }

        point_angle_var_2D = 0.0;

        for (int idx = 0; idx < DATASET_SIZE_2D; idx++)
        {
            int data_idx = (DATASET_SIZE_2D - 1 - idx);
            actualDistance = (cloudSetBuffer_2D[data_idx]);
            
            point_angle_2D = (float)(((HORIZONTAL_ANGLE / 2 * -1) + point_angle_var_2D) * RADIAN);
            point_angle_var_2D += angleStep_2D;

            //actualDistance = (cloudSetBuffer_2D[idx]);
            actualX = (sin(point_angle_2D) * actualDistance);
            actualY = (cos(point_angle_2D) * actualDistance); // depth

            tempX_2D = actualX;
            tempY_2D = actualY;
            actualX = (tempX_2D * cos(angleRadian)) + (tempY_2D * -sin(angleRadian));
            actualY = (tempX_2D * sin(angleRadian)) + (tempY_2D * cos(angleRadian));

            scan_2D.get()->points[idx].x = actualX * DIVISOR;
            scan_2D.get()->points[idx].y = -actualY * DIVISOR;
            scan_2D.get()->points[idx].z = 0.0;
            
            if (cloudSetBuffer_2D[idx] < (float)BASE_DEPTH_2D)
            {
                scan_2D.get()->points[idx].r = 255;
                scan_2D.get()->points[idx].g = 255;
                scan_2D.get()->points[idx].b = 0;
                scan_2D.get()->points[idx].a = 255;

                scan_laser->ranges[idx] = cloudSetBuffer_2D[data_idx] * DIVISOR;
            }
            else
            {
                scan_2D.get()->points[idx].a = 0;

                scan_laser->ranges[idx] = 0.0;
            }
        }

        pcl_conversions::toPCL(ros::Time::now(), scan_2D->header.stamp);
        pub_2D.publish(scan_2D);

        current_time_laser = ros::Time::now();
        time_diff_laser = (current_time_laser - last_time_laser).toSec() * 1e-3;
        scan_laser->scan_time = time_diff_laser;
        scan_laser->time_increment = (time_diff_laser / (float)(DATASET_SIZE_2D - 1));
        scan_laser->header.stamp = current_time_laser;//ros::Time(0);
        last_time_laser = current_time_laser;
        pub_scan.publish(scan_laser);

        drawing = false; 
    }
}

int distanceCnt_3D, index_3D;
float tempX_3D, tempY_3D, tempD_3D;
float x_3D, y_3D, h_3D, fh_3D, dRatio_3D, aRatio_3D, focalRatio_3D, tanA1_3D, tanA2_3D;
float verticalA, horizontalA, verticalA_Single, horizontalA_Half, originalA_H, originalA_V, differenceA_H, differenceA_V;
uint32_t rgb_3D;
uint8_t cloudScatter_3D()
{
    if (!drawing)
    {
        drawing = true;

        switch (currentBuffer)
        {
            case 0x00:
                memcpy(cloudBuffer, &bufferPtr02[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_3D);
                break;
            case 0x01:
                memcpy(cloudBuffer, &bufferPtr01[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_3D);
                break;
        }
        
        distanceCnt_3D = 0;
        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_3D - PAYLOAD_SIZE) - 1; dataLength+=3)
        {
            FIRST = cloudBuffer[dataLength];
            SECOND = cloudBuffer[dataLength + 1];
            THIRD = cloudBuffer[dataLength + 2];

            data1 = (float)((FIRST << 4) | (SECOND >> 4));
            data2 = (float)(((SECOND & 0xf) << 8) | THIRD);

            cloudSetBuffer[distanceCnt_3D++] = data1;
            cloudSetBuffer[distanceCnt_3D++] = data2;
        }

        horizontalA_Half = (float)HORIZONTAL_ANGLE / 2;
        verticalA_Single = (float)VERTICAL_ANGLE / centerX_3D;
        originalA_H = (atan(centerX_3D / FOCAL_LENGTH) * (HALF_ANGLE / MATH_PI));
        originalA_V = (atan(centerY_3D / FOCAL_LENGTH) * (HALF_ANGLE / MATH_PI));
        differenceA_H = (horizontalA_Half / originalA_H);
        differenceA_V = ((VERTICAL_ANGLE / 2) / originalA_V);
        
        index_3D = 0;
        for (int yy = 0; yy < height; yy++)
        {
            for (int xx = 0; xx < width; xx++)
            {
                index_3D = (xx + (yy * width));

                x_3D = abs(xx - centerX_3D);
                y_3D = abs(yy - centerY_3D);
                
                tanA1_3D = y_3D / x_3D;
                h_3D = (y_3D == 0 ? x_3D : (y_3D / sin(atan(tanA1_3D))));

                tanA2_3D = FOCAL_LENGTH / h_3D;
                tempD_3D = FOCAL_LENGTH / sin(atan(tanA2_3D));

                dRatio_3D = (cloudSetBuffer[index_3D] / tempD_3D);
                actualDistance = (FOCAL_LENGTH * dRatio_3D);

                actualX = (xx - centerX_3D) * (actualDistance / FOCAL_LENGTH) * differenceA_H;
                actualY = -(yy - centerY_3D) * (actualDistance / FOCAL_LENGTH) * differenceA_V;
                
                // Rotate the axis of clouds to the right
                tempX_3D = actualX;
                tempY_3D = actualDistance;
                actualX = (tempX_3D * cos(angleRadian)) + (tempY_3D * -sin(angleRadian));
                actualDistance = (tempX_3D * sin(angleRadian)) + (tempY_3D * cos(angleRadian));
                
                scan_3D.get()->points[index_3D].x = actualX * DIVISOR;
                scan_3D.get()->points[index_3D].y = actualDistance * DIVISOR;
                scan_3D.get()->points[index_3D].z = (actualY + HALF_ANGLE) * DIVISOR;

                // Determine a cloud color based on the distance

                if (cloudSetBuffer[index_3D] > BASE_DEPTH_3D)
                {
                    if (cloudSetBuffer[index_3D] == SATURATION_VALUE_3D)
                    {
                        scan_3D.get()->points[index_3D].r = 210;
                        scan_3D.get()->points[index_3D].g = 0;
                        scan_3D.get()->points[index_3D].b = 255;
                        scan_3D.get()->points[index_3D].a = 255;
                    }
                    else
                    {
                        scan_3D.get()->points[index_3D].a = std::numeric_limits<float>::infinity();
                    }
                }
                else
                {
                    rgb_3D = colorArray[(int)tempY_3D % colorArray.size()];
                    scan_3D.get()->points[index_3D].rgb = *reinterpret_cast<float*>(&rgb_3D);
                    scan_3D.get()->points[index_3D].a = 255;
                }
            }
        }

        pcl_conversions::toPCL(ros::Time::now(), scan_3D->header.stamp);
        pub_3D.publish(scan_3D);

        drawing = false;
    }
}

int VERSION_NUM, FREQUENCY_LEVEL, PULSE_CONTROL, PULSE_DURATION;
void running()
{
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string port;
    int baud_rate;
    std::string frame_id;

    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    priv_nh.param("baud_rate", baud_rate, 3000000);
    priv_nh.param("frame_id", frame_id, std::string("laser_link"));

    priv_nh.param("version", VERSION_NUM, 0);
    priv_nh.param("frequency", FREQUENCY_LEVEL, 0);
    priv_nh.param("pulse_control", PULSE_CONTROL, 0);
    priv_nh.param("duration", PULSE_DURATION, 0);


    ROS_INFO("FREQUENCY: %d (%x) / PULSE CONTROL: %d, DURATION: %d (%x)", \
    FREQUENCY_LEVEL, FREQUENCY_LEVEL, \
    PULSE_CONTROL, PULSE_DURATION, PULSE_DURATION);

    colorBuffer();

    pub_scan = nh.advertise<sensor_msgs::LaserScan>("scan_laser", SIZE_MAX);
    pub_2D = nh.advertise<sensor_msgs::PointCloud2>("scan_2D", 1);
    pub_3D = nh.advertise<sensor_msgs::PointCloud2>("scan_3D", 1);
    scan_2D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_3D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_laser = sensor_msgs::LaserScan::Ptr(new sensor_msgs::LaserScan);

    // LaserScan
    scan_laser->header.frame_id = frame_id;
    
    // PointCloud2 for 2D
    scan_2D.get()->is_dense = false;
    scan_2D.get()->header.frame_id = frame_id;

    // PointCloud2 for 3D
    scan_3D.get()->width = 160;
    scan_3D.get()->height = 60;
    scan_3D.get()->is_dense = false;
    scan_3D.get()->header.frame_id = frame_id;

    width = (float)(scan_3D.get()->width);
    height = (float)(scan_3D.get()->height);
    
    centerX_3D = (width / 2);
    centerY_3D = (height / 2);

    actualDistance = 0.0;
    length = 0.0;
    currentBuffer = 0x00;
    drawing = false;

    bufferPtr = new uint8_t[SIZE_MAX];
    uint8_t* result;
    uint8_t parser, inProgress = 0x00;
	int bytes_transferred;
	size_t sizePos = 2;

    current_time_laser = ros::Time::now();
    last_time_laser = ros::Time::now();

    bool buffer_setup_2d = false;
    bool buffer_setup_3d = false;
    
    boost::asio::io_service io;
    try
    {
        cyglidar_pcl_driver::cyglidar_pcl laser(port, baud_rate, io);
        laser.packet_run(VERSION_NUM);
        ros::Duration(1.0).sleep();
        laser.packet_frequency(FREQUENCY_LEVEL);
        ros::Duration(1.0).sleep();
        laser.packet_pulse(VERSION_NUM, PULSE_CONTROL, PULSE_DURATION);

        int DATA_1 = 4, DATA_2 = 3;

        while (ros::ok())
        {
            if (inProgress == 0x00)
            {
				inProgress = 0x01;

                result = laser.poll(VERSION_NUM);
                bytes_transferred = (int)(result[SIZE_MAX] << 8 | result[SIZE_MAX + 1]);

                if (bytes_transferred > 0)
                {
                    for (int i = 0; i < bytes_transferred; i++)
                    {
                        parser = CygParser(bufferPtr, result[i]);

                        switch (parser)
                        {
                            case 0x01:
                                //ROS_ERROR("DATA >> %x, %x, %x, %x, %x, %x",\
                                bufferPtr[0], bufferPtr[1], bufferPtr[2], bufferPtr[3], bufferPtr[4], bufferPtr[5]);
                                switch (bufferPtr[5])
                                {
                                    case 0x01: // 2D
                                        if (!buffer_setup_2d)
                                        {
                                            DATABUFFER_SIZE_2D = (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]) + PAYLOAD_SIZE;
                                            DATASET_SIZE_2D = (int)((float)(DATABUFFER_SIZE_2D - PAYLOAD_SIZE) / 2.0);
                                            angleStep_2D = (HORIZONTAL_ANGLE / (DATASET_SIZE_2D - 1));

                                            cloudBuffer_2D = new uint8_t[DATABUFFER_SIZE_2D - PAYLOAD_SIZE];
                                            cloudSetBuffer_2D = new float[DATASET_SIZE_2D];

                                            scan_laser->angle_min = -((double)HORIZONTAL_ANGLE / 2) * RADIAN;
                                            scan_laser->angle_max = ((double)HORIZONTAL_ANGLE / 2) * RADIAN;
                                            scan_laser->angle_increment = (angleStep_2D * RADIAN);
                                            scan_laser->range_min = 0.1;
                                            scan_laser->range_max = (DISTANCE_MAX_2D * DIVISOR);
                                            scan_laser->ranges.resize(DATASET_SIZE_2D);
                                            scan_2D.get()->points.resize(DATASET_SIZE_2D);

                                            buffer_setup_2d = true;
                                        }
                                        
                                        bufferPtr_2D = &bufferPtr[0];
                                        cloudScatter_2D();
                                        //ROS_ERROR("2D ==> %d [%d, %d]",\
                                        (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]), DATABUFFER_SIZE_2D, DATASET_SIZE_2D);
                                        break;
                                    case 0x08: // 3D
                                        if (!buffer_setup_3d)
                                        {
                                            DATABUFFER_SIZE_3D = (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]) + PAYLOAD_SIZE;
                                            float BYTESET_RATIO_3D = (2.0 / 3.0);
                                            DATASET_SIZE_3D = (int)((float)(DATABUFFER_SIZE_3D - PAYLOAD_SIZE) * BYTESET_RATIO_3D);
                                            cloudBuffer = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];
                                            cloudSetBuffer = new float[DATASET_SIZE_3D];

                                            scan_3D.get()->points.resize(DATASET_SIZE_3D);

                                            buffer_setup_3d = true;
                                        }

                                        switch (currentBuffer)
                                        {
                                            case 0x00:
                                                bufferPtr01 = &bufferPtr[0];
                                                currentBuffer = 0x01;
                                                break;
                                            case 0x01:
                                                bufferPtr02 = &bufferPtr[0];
                                                currentBuffer = 0x00;
                                                break;
                                        }
                                        cloudScatter_3D();
                                        //ROS_ERROR("3D ==> %d [%d, %d]",\
                                        (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]), DATABUFFER_SIZE_3D, DATASET_SIZE_3D);
                                        break;
                                }
                                break;
                            default:
                                break;
                        }
                    }
                }

				inProgress = 0x00;
            }
        }
        laser.close();
    }
    catch (boost::system::system_error ex)
    {
        ROS_ERROR("[Error] instantiating laser object. \
        Are you sure you have the correct port and baud rate? \
        Error was %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "line_laser");

    std::thread first(running);
    first.join();

    return 0;
}
