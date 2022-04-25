#include <sl/Camera.hpp>
#include <glm/vec4.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <ctime>
#include <sstream>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <experimental/filesystem>
#include "json.hpp"

namespace fs = std::experimental::filesystem;

inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}

void get_pcl_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_cloud, sl::Camera &zed)
{
    sl::Mat zed_pc;
    zed.retrieveMeasure(zed_pc, sl::MEASURE::XYZRGBA, sl::MEM::GPU);
    float *p_zed_pc = zed_pc.getPtr<float>();
    int index = 0;
    // Check and adjust points for PCL format
    for (auto &it : p_pcl_cloud->points) {
        float X = p_zed_pc[index];
        it.x = X;
        it.y = p_zed_pc[index + 1];
        it.z = p_zed_pc[index + 2];
        it.rgb = convertColor(p_zed_pc[index + 3]); // Convert a 32bits float into a pcl .rgb format
        index += 4;
    }
}

std::string getTimestampStr() {
    time_t time = std::time(nullptr);
    tm tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return oss.str();
}

void handler(int s) {
    //close files and shtuff
    exit(s);
}

int main(int argc, char **argv) {
    // Create a ZED camera object
    sl::Camera zed;
    // Set configuration parameters (@TODO: confirm)
    sl::InitParameters init_params;
    init_params.sdk_verbose = true; // Disable verbose mode
    init_params.camera_resolution = sl::RESOLUTION::VGA;
    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        exit(-1);
    }
    //Zed point cloud
    //PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                p_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Gets ZED point cloud and converts to PCL point cloud

    //TRYING TO DO SIGINT stuff
    signal(SIGINT, handler);
    //

    //start loop
    get_pcl_cloud(p_pcl_cloud, zed);
    //Data stream
    pcl::PCDWriter writer;
    fs::path tempDir = fs::temp_directory_path();
    writer.writeBinaryCompressed(tempDir.string() + "temp.pcd", *p_pcl_cloud);

    sl::SensorsData sensorData;
    zed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

    ifstream fin(tempDir.string() + "temp.pcd", std::ios::binary);
    std::stringstream buffer;
    buffer << fin.rdbuf();
    
    nlohmann::json jsonData = {
                            {"timestamp", getTimestampStr()},    
                            {"magnetometer", {
                                {"state", sensorData.magnetometer.magnetic_heading_state},
                                {"heading", sensorData.magnetometer.magnetic_heading}
                               }}, 
                               {"angular velocity", {
                                   {"x", sensorData.imu.angular_velocity.x},
                                   {"y", sensorData.imu.angular_velocity.y},
                                   {"z", sensorData.imu.angular_velocity.z}
                               }},
                               {"linear acceleration", {
                                   {"x", sensorData.imu.linear_acceleration.x},
                                   {"y", sensorData.imu.linear_acceleration.y},
                                   {"z", sensorData.imu.linear_acceleration.z}
                               }},
                               {"barometer", sensorData.barometer.pressure},
                               {"temperature", sensorData.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT]},
                               {"point cloud binary", buffer.str()}};

    //add json to a file dir


    //loop

    //end loop


    // Close the camera
    zed.close();
    return 0;
}