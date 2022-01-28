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

uint8_t *cloudBuffer;
float *cloudSetBuffer;
uint8_t *cloudBuffer_2D;
float *cloudSetBuffer_2D;

uint8_t* bufferPtr_2D;
uint8_t* bufferPtr01;
uint8_t* bufferPtr02;
uint8_t* bufferPtr;

float angleRadian = RADIAN * -RIGHT_ANGLE;      // for angular rotation
float centerX_3D, centerY_3D;

const int ImageWidth = 160;
const int ImageHeight = 60;

uint8_t currentBuffer;

std::pair<double, double> DistortionHashMap[ImageWidth * ImageHeight];

void MakeDistortionHashMap(std::pair<double, double>* DistortionHashMap, const int w, const int h)
{
	double x_nu, y_nu, r2, r4, coeff, x_nd, y_nd, x_pd, y_pd;

	double dWidth = static_cast<double>(w);
	double dHeight = static_cast<double>(h);

	for (double dx = 0; dx < dWidth; dx += 1.0)
	{
		for (double dy = 0; dy < dHeight; dy += 1.0)
		{
			// pixel coordinate system --> normal coordinate system
			x_nu = (dx - CameraIntrinsicParameters::cx) / CameraIntrinsicParameters::f;     // x_nu : Undistorted normal coordinate x.
			y_nu = (dy - CameraIntrinsicParameters::cy) / CameraIntrinsicParameters::f;     // y_nu : Undistorted normal coordinate y.

			r2 = (x_nu * x_nu) + (y_nu * y_nu); // r^2
			r4 = r2 * r2;                       // r^4

			//use only distortion coefficients k1 and k2
			coeff = 1.0 + (LensDistortionCoefficients::k1 * r2) + (LensDistortionCoefficients::k2 * r4);

			x_nd = coeff * x_nu;    // x_nd : Distorted normal coordinate x.
			y_nd = coeff * y_nu;    // y_nd : Distorted normal coordinate y.

			// normal coordinate system --> pixel coordinate system
			x_pd = (x_nd * CameraIntrinsicParameters::f) + CameraIntrinsicParameters::cx;  // x_pd : Distorted pixel coordinate x.
			y_pd = (y_nd * CameraIntrinsicParameters::f) + CameraIntrinsicParameters::cy;  // y_pd : Distorted pixel coordinate y.

			DistortionHashMap[(int)(dx + (dy * dWidth))].first = x_pd;
			DistortionHashMap[(int)(dx + (dy * dWidth))].second = y_pd;
		}
	}
}

inline bool isValidCoordinate(const double x, const double y, const double maxX, const double maxY)
{
    if(x < 0 || y < 0 || maxX - 1.0 < x || maxY - 1.0 < y) return false;
    return true;
}




static std::vector<uint32_t> colorArray;
void colorBuffer()
{
	uint8_t r_setup = 254;
	uint8_t g_setup = 0;
	uint8_t b_setup = 0;

	// Iterate for-loop of adding RGB value to an array
	for (int i = 0; i < 3; i++)
	{
		for (int colorCnt = 0; colorCnt < COLOR_MAX; colorCnt++)
		{
			switch (i)
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

			uint32_t rgb_setup = ((uint32_t)r_setup << 16 | (uint32_t)g_setup << 8 | (uint32_t)b_setup);
			colorArray.push_back(rgb_setup);
		}
	}
}

bool drawing = false;
float tempX_2D, tempY_2D;
void cloudScatter_2D(ros::Time start, ros::Time end, const double ANGLE_STEP_2D)
{
    if (!drawing)
    {
        // Make the drawing state TRUE so that it's not interrupted while drawing
        drawing = true;

        // Copy the memory of the rawdata array to the working array
        memcpy(cloudBuffer_2D, &bufferPtr_2D[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_2D);

        int distanceCnt_2D = 0;

        // Convert rawdata to measured distance appending both separated data (LSB and MSB)
        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_2D - PAYLOAD_SIZE) - 1; dataLength += 2)
        {
            uint8_t MSB = cloudBuffer_2D[dataLength];
            uint8_t LSB = cloudBuffer_2D[dataLength + 1];

            float rawDistance = (float)((MSB << 8) | LSB);

            cloudSetBuffer_2D[distanceCnt_2D++] = rawDistance;
        }

        // Update values of LaserScan
        double scan_duration = (start - end).toSec() * 1e-3;

        scan_laser->header.stamp = start;
        scan_laser->scan_time = scan_duration;
        scan_laser->time_increment = (scan_duration / (float)(DATASET_SIZE_2D - 1));

        // Initialize the current point of FoV at which a given distance is measured
        float point_angle_var_2D = 0.0;

        // Loop for drawing distances
        for (int idx = 0; idx < DATASET_SIZE_2D; idx++)
        {
            // Reverse data order of the array
            int data_idx = (DATASET_SIZE_2D - 1 - idx);
            float actualDistance = (cloudSetBuffer_2D[data_idx]);

            // Get the latest angle of the distance
            float point_angle_2D = (float)(((-HORIZONTAL_ANGLE / 2)+ point_angle_var_2D) * RADIAN);
            point_angle_var_2D += ANGLE_STEP_2D;

            // Calculate both X and Y coordinates
            float actualX = (sin(point_angle_2D) * actualDistance);
            float actualY = (cos(point_angle_2D) * actualDistance); // depth

            // Rotate the dataset 90 clockwise about the origin
            tempX_2D = actualX;
            tempY_2D = actualY;
            actualX = (tempX_2D * cos(angleRadian)) + (tempY_2D * -sin(angleRadian));
            actualY = (tempX_2D * sin(angleRadian)) + (tempY_2D * cos(angleRadian));

            // Store the computed coordinates to PointCloud2 and LaserScan
            scan_2D.get()->points[idx].x = actualX * MM2M;
            scan_2D.get()->points[idx].y = -actualY * MM2M;
            scan_2D.get()->points[idx].z = 0.0;

            // Turn data invisible when it's greater than the maximum
            if (cloudSetBuffer_2D[data_idx] < (float)BASE_DEPTH_2D)
            {
                scan_2D.get()->points[idx].r = 255;
                scan_2D.get()->points[idx].g = 255;
                scan_2D.get()->points[idx].b = 0;
                scan_2D.get()->points[idx].a = 255;
                scan_laser->ranges[idx] = cloudSetBuffer_2D[data_idx] * MM2M;
            }
            else
            {
                scan_2D.get()->points[idx].a = 0;
                scan_laser->ranges[idx] = std::numeric_limits<float>::infinity();
            }
        }

        // Publish messages on both topics
        pcl_conversions::toPCL(ros::Time::now(), scan_2D->header.stamp);
        pub_2D.publish(scan_2D);

        pub_scan.publish(scan_laser);

        // Allow the latest dataset to enter in this constructor
        drawing = false;
    }
}

void cloudScatter_3D()
{
    if (!drawing)
    {
        // Make the drawing state TRUE so that it's not interrupted while drawing
        drawing = true;

        // Copy the memory of the rawdata array to the working array
        switch (currentBuffer)
        {
            case 0x00:
                cloudBuffer = &bufferPtr02[0];      // ref. to output buffer
                currentBuffer = 0x01;               // buffer swap
                break;
            case 0x01:
                cloudBuffer = &bufferPtr01[0];
                currentBuffer = 0x00;
                break;
        }

        // Convert rawdata to measured distance appending both separated data (LSB and MSB)
        int distanceCnt_3D = 0;
        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_3D - PAYLOAD_SIZE) - 1; dataLength+=3)
        {
            uint8_t FIRST = cloudBuffer[dataLength];
            uint8_t SECOND = cloudBuffer[dataLength + 1];
            uint8_t THIRD = cloudBuffer[dataLength + 2];

            // data1 is a combination of the first and left half of second
            // data2 is of the right half of second and third
            float data1 = (float)((FIRST << 4) | (SECOND >> 4));
            float data2 = (float)(((SECOND & 0xf) << 8) | THIRD);

            cloudSetBuffer[distanceCnt_3D++] = data1;
            cloudSetBuffer[distanceCnt_3D++] = data2;
        }

        // Loop for drawing distances
        int index_3D = 0;
        for (int yy = 0; yy < ImageHeight; yy++)
        {
            for (int xx = 0; xx < ImageWidth; xx++)
            {
                index_3D = (xx + (yy * ImageWidth));
                    double UnDis_x = DistortionHashMap[index_3D].first;
                    double UnDis_y = DistortionHashMap[index_3D].second;
                if(isValidCoordinate(UnDis_x, UnDis_y, (double)ImageWidth, (double)ImageHeight))
                {
                    int UnDisIndex_x = static_cast<int>(UnDis_x );
                    int UnDisIndex_y = static_cast<int>(UnDis_y );
                    int UnDisIndex = UnDisIndex_x + (UnDisIndex_y * ImageWidth);

                    // Get the tangent of an angle and the hypotenuse
                    float x_3D = static_cast<float>(UnDis_x - CameraIntrinsicParameters::cx);
                    float y_3D = static_cast<float>(UnDis_y - CameraIntrinsicParameters::cy);
                    float ll_3D = (x_3D * x_3D) + (y_3D * y_3D);


                    // Get the tangent of an angle and the hypotenuse about focal length
                    float hh_3D = ll_3D + (CameraIntrinsicParameters::f*CameraIntrinsicParameters::f);

                    // Calculate the ratio between the distance and the hypotenuse right above
                    float dRatio_3D = (cloudSetBuffer[UnDisIndex] / sqrt(hh_3D));
                    float actualDistance = (FOCAL_LENGTH * dRatio_3D);

                    // Calculate both X and Y coordinates
                    float actualX = (xx - centerX_3D) * (actualDistance / FOCAL_LENGTH);
                    float actualY = -(yy - centerY_3D) * (actualDistance / FOCAL_LENGTH);

                    // Rotate the dataset 90 clockwise about the origin
                    float tempX_3D = actualX;
                    float tempY_3D = actualDistance;

                    actualX = (tempX_3D * cos(angleRadian)) + (tempY_3D * -sin(angleRadian));
                    actualDistance = (tempX_3D * sin(angleRadian)) + (tempY_3D * cos(angleRadian));

                    // Store the computed coordinates to PointCloud2 and LaserScan
                    scan_3D.get()->points[UnDisIndex].x = actualX * MM2M;
                    scan_3D.get()->points[UnDisIndex].y = actualDistance * MM2M;
                    scan_3D.get()->points[UnDisIndex].z = (actualY + HALF_ANGLE) * MM2M;

                    // Turn data invisible when it's greater than the maximum
                    if (cloudSetBuffer[UnDisIndex] < BASE_DEPTH_3D)
                    {
                        uint32_t rgb_3D = colorArray[(int)tempY_3D % colorArray.size()];
                        scan_3D.get()->points[UnDisIndex].rgb = *reinterpret_cast<float*>(&rgb_3D);
                        scan_3D.get()->points[UnDisIndex].a = 255;
                    }
                    else
                    {
                        scan_3D.get()->points[UnDisIndex].a = 0;
                    }
                }
            }
        }

        // Publish a message on PointCloud2 topic
        pcl_conversions::toPCL(ros::Time::now(), scan_3D->header.stamp);
        pub_3D.publish(scan_3D);

        // Allow the latest dataset to enter in this constructor
        drawing = false;
    }
}

int VERSION_NUM, FREQUENCY_LEVEL, PULSE_CONTROL, PULSE_DURATION;
void running()
{
    // Create node handlers and local variables
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string port;
    int baud_rate;
    std::string frame_id;

    // Assign the given values by users
    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    priv_nh.param("baud_rate", baud_rate, 3000000);
    priv_nh.param("frame_id", frame_id, std::string("laser_link"));

    priv_nh.param("version", VERSION_NUM, 0);
    priv_nh.param("frequency", FREQUENCY_LEVEL, 0);
    priv_nh.param("pulse_control", PULSE_CONTROL, 0);
    priv_nh.param("duration", PULSE_DURATION, 0);

    // Check whether the values are applied properly
    ROS_INFO("FREQUENCY: %d (%x) / PULSE CONTROL: %d, DURATION: %d (%x)", \
    FREQUENCY_LEVEL, FREQUENCY_LEVEL, \
    PULSE_CONTROL, PULSE_DURATION, PULSE_DURATION);

    // Call the following function so as to store colors to draw 3D data
    colorBuffer();

    MakeDistortionHashMap(DistortionHashMap, ImageWidth, ImageHeight);

    // Initialize variables
    pub_scan = nh.advertise<sensor_msgs::LaserScan>("scan_laser", SCAN_MAX_SIZE);
    pub_2D = nh.advertise<sensor_msgs::PointCloud2>("scan_2D", 1);
    pub_3D = nh.advertise<sensor_msgs::PointCloud2>("scan_3D", 1);
    scan_2D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_3D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_laser = sensor_msgs::LaserScan::Ptr(new sensor_msgs::LaserScan);


    float fIw = static_cast<float>(ImageWidth);
    float fIh = static_cast<float>(ImageHeight);
    centerX_3D = (fIw / 2.0f) - 3.0f;
    centerY_3D = (fIh / 2.0f) - 3.0f;

    currentBuffer = 0x00;
    drawing = false;

    bufferPtr = new uint8_t[SCAN_MAX_SIZE];
    uint8_t* result;
    uint8_t parser, inProgress = 0x00;
	int bytes_transferred;

    ros::Time current_time_laser = ros::Time::now();
    ros::Time last_time_laser = ros::Time::now();

    bool buffer_setup_2d = false;
    bool buffer_setup_3d = false;

    // LaserScan
    scan_laser->header.frame_id = frame_id;

    // PointCloud2 for 2D
    scan_2D.get()->is_dense = false;
    scan_2D.get()->header.frame_id = frame_id;

    // PointCloud2 for 3D
    scan_3D.get()->width = ImageWidth;
    scan_3D.get()->height = ImageHeight;
    scan_3D.get()->is_dense = false;
    scan_3D.get()->header.frame_id = frame_id;

    boost::asio::io_service io;
    try
    {
        // Open the port
        cyglidar_pcl_driver::cyglidar_pcl laser(port, baud_rate, io);

        // Send packets
        laser.packet_run(VERSION_NUM);
        ros::Duration(1.0).sleep();         // sleep for a sec, by the duration
        laser.packet_frequency(FREQUENCY_LEVEL);
        ros::Duration(1.0).sleep();
        laser.packet_pulse(VERSION_NUM, PULSE_CONTROL, PULSE_DURATION);

        // Create variables used to organize data before drawing
        int DATA_1 = 4, DATA_2 = 3;
        double ANGLE_STEP_2D, ANGLE_POINT_2D;

        // Keep receiving data unless the process is dead
        while (ros::ok())
        {
            // Only allowed in case of not using the working array called bufferPtr
            if (inProgress == 0x00)
            {
				inProgress = 0x01;

                result = laser.poll(VERSION_NUM);
                bytes_transferred = (int)(result[SCAN_MAX_SIZE] << 8 | result[SCAN_MAX_SIZE + 1]);

                if (bytes_transferred > 0)
                {
                    for (int i = 0; i < bytes_transferred; i++)
                    {
                        // Check on the validity of dataset
                        parser = CygParser(bufferPtr, result[i]);

                        switch (parser)
                        {
                            case 0x01:
                                switch (bufferPtr[5])
                                {
                                    case 0x01: // 2D
                                        if (!buffer_setup_2d)
                                        {
                                            DATABUFFER_SIZE_2D = (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]) + PAYLOAD_SIZE;
                                            DATASET_SIZE_2D = (int)((float)(DATABUFFER_SIZE_2D - PAYLOAD_SIZE) / 2.0);

                                            ANGLE_STEP_2D = (((double)BASE_ANGLE_2D / (double)(DATASET_SIZE_2D - 1)));
                                            ANGLE_POINT_2D = ((double)BASE_ANGLE_2D / 2) * RADIAN;

                                            cloudBuffer_2D = new uint8_t[DATABUFFER_SIZE_2D - PAYLOAD_SIZE];
                                            cloudSetBuffer_2D = new float[DATASET_SIZE_2D];

                                            scan_laser->header.frame_id = frame_id;

                                            scan_laser->angle_min = -ANGLE_POINT_2D;
                                            scan_laser->angle_max = ANGLE_POINT_2D;
                                            scan_laser->angle_increment = ANGLE_STEP_2D * RADIAN;

                                            scan_laser->range_min = 0.1;
                                            scan_laser->range_max = ((double)BASE_DEPTH_2D * MM2M);

                                            scan_laser->ranges.resize(DATASET_SIZE_2D);
                                            scan_laser->intensities.resize(DATASET_SIZE_2D);

                                            scan_2D.get()->points.resize(DATASET_SIZE_2D);

                                            buffer_setup_2d = true;
                                        }
                                        if (buffer_setup_2d)
                                        {
                                            last_time_laser = ros::Time::now();
                                            bufferPtr_2D = &bufferPtr[0];

                                            cloudScatter_2D(current_time_laser, last_time_laser, ANGLE_STEP_2D);
                                        }
                                        break;
                                    case 0x08: // 3D
                                        if (!buffer_setup_3d)
                                        {
                                            DATABUFFER_SIZE_3D = (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]) + PAYLOAD_SIZE;
                                            float BYTESET_RATIO_3D = (2.0 / 3.0);
                                            DATASET_SIZE_3D = (int)((float)(DATABUFFER_SIZE_3D - PAYLOAD_SIZE) * BYTESET_RATIO_3D);
                                            // cloudBuffer = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];
                                            bufferPtr01 = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];       // memory allocation to buffer A
                                            bufferPtr02 = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];       // memory allocation to buffer B
                                            cloudSetBuffer = new float[DATASET_SIZE_3D];

                                            scan_3D.get()->points.resize(DATASET_SIZE_3D);

                                            buffer_setup_3d = true;
                                        }
                                        if (buffer_setup_3d)
                                        {
                                            switch (currentBuffer)
                                            {
                                                case 0x00:
                                                    memcpy(bufferPtr01, bufferPtr, sizeof(uint8_t) * DATABUFFER_SIZE_3D);   // copy to buffer
                                                    break;
                                                case 0x01:
                                                    memcpy(bufferPtr02, bufferPtr, sizeof(uint8_t) * DATABUFFER_SIZE_3D);
                                                    break;
                                            }
                                            cloudScatter_3D();
                                        }
                                        break;
                                }
                                break;
                            case 0x00: // passed through header 1
                                current_time_laser = ros::Time::now();
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
        delete bufferPtr01;
        delete bufferPtr02;
        delete cloudSetBuffer;
        delete cloudBuffer_2D;
        delete cloudSetBuffer_2D;
    }
    catch (const boost::system::system_error& ex)
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
