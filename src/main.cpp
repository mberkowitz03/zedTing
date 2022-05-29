#include <ctime>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sl/Camera.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

sl::Mat zed_pc;
uint64_t frame = 0;
volatile sig_atomic_t stop = 0;

float convertColor(float colorIn)
{
    uint32_t color_uint = *(uint32_t *)&colorIn;
    unsigned char *color_uchar = (unsigned char *)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float *>(&color_uint);
}

void get_pcl_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_cloud, sl::Camera &zed)
{
    sl::ERROR_CODE err = zed.grab();
    if (err == sl::ERROR_CODE::SUCCESS)
    {
        err = zed.retrieveMeasure(zed_pc, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
        if (err == sl::ERROR_CODE::SUCCESS)
        {
            float *p_zed_pc = zed_pc.getPtr<float>();
            int index = 0;
            // Check and adjust points for PCL format
            for (auto &it : p_pcl_cloud->points)
            {
                it.x = p_zed_pc[index + 0];
                it.y = p_zed_pc[index + 1];
                it.z = p_zed_pc[index + 2];
                it.rgb = convertColor(p_zed_pc[index + 3]); // Convert a 32bits float into a pcl .rgb format
                index += 4;
            }
        }
        else
        {
            std::cerr << "Failed to retrieve measure: " << err << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        std::cerr << "Failed to grab: " << err << std::endl;
        exit(EXIT_FAILURE);
    }
}

std::string getTimestampStr()
{
    time_t time = std::time(nullptr);
    tm tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S");
    return oss.str();
}

void handler(int s)
{
    // close files and shtuff
    stop = 1;
}

int main(int argc, char **argv)
{
    std::string dirName = "../recordings";
    if (!fs::is_directory(dirName))
    {
        fs::create_directory(dirName);
    }
    sl::Camera zed;
    // Set configuration parameters (@TODO: confirm)
    sl::InitParameters init_params;
    init_params.sdk_verbose = true;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 15;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        std::cerr << "Failed to open ZED: " << err << std::endl;
        exit(EXIT_FAILURE);
    }

    // Zed point cloud
    // PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_cloud->points.resize(zed.getCameraInformation().camera_configuration.resolution.area());
    pcl::PCDWriter writer;

    // TRYING TO DO SIGINT stuff
    signal(SIGINT, handler);

    // start loop
    while (!stop)
    {
        get_pcl_cloud(p_pcl_cloud, zed);

        sl::SensorsData sensorData;
        zed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

        std::string timeStamp = getTimestampStr();
        std::string fileName = dirName + "/" + timeStamp + " #" + std::to_string(++frame) + ".pcd";
        writer.writeBinaryCompressed(fileName, *p_pcl_cloud);
        std::cout << "Wrote: " << fileName << std::endl;
        std::ifstream pcdFile(fileName);
        std::stringstream pcdStream;
        pcdStream << pcdFile.rdbuf();

        // TODO: insert into PCD directly, don't bother with json

        // nlohmann::json jsonData = {
        //                         {"timestamp", timeStamp},
        //                         {"magnetometer", {
        //                             {"state", sensorData.magnetometer.magnetic_heading_state},
        //                             {"heading", sensorData.magnetometer.magnetic_heading}
        //                         }},
        //                         {"angular velocity", {
        //                             {"x", sensorData.imu.angular_velocity.x},
        //                             {"y", sensorData.imu.angular_velocity.y},
        //                             {"z", sensorData.imu.angular_velocity.z}
        //                         }},
        //                         {"linear acceleration", {
        //                             {"x", sensorData.imu.linear_acceleration.x},
        //                             {"y", sensorData.imu.linear_acceleration.y},
        //                             {"z", sensorData.imu.linear_acceleration.z}
        //                         }},
        //                         {"barometer", sensorData.barometer.pressure},
        //                         {"temperature", sensorData.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT]},
        //                         {"point cloud binary", buffer.str()}};
    }

    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}