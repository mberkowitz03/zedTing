#include <ctime>
#include <csignal>
#include <iostream>
#include <sl/Camera.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

sl::Mat zed_pc;
uint64_t frame = 0;
volatile sig_atomic_t stop = 0;

float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t*) &colorIn;
    auto* color_uchar = (unsigned char*) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float*>(&color_uint);
}

void get_pcl_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& p_pcl_cloud, sl::Camera& zed) {
    sl::ERROR_CODE err = zed.grab();
    if (err == sl::ERROR_CODE::SUCCESS) {
        err = zed.retrieveMeasure(zed_pc, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
        if (err == sl::ERROR_CODE::SUCCESS) {
            auto* p_zed_pc = zed_pc.getPtr<float>();
            int index = 0;
            // Check and adjust points for PCL format
            for (auto& it: p_pcl_cloud->points) {
                it.x = p_zed_pc[index + 0];
                it.y = p_zed_pc[index + 1];
                it.z = p_zed_pc[index + 2];
                it.rgb = convertColor(p_zed_pc[index + 3]); // Convert a 32bits float into a pcl .rgb format
                index += 4;
            }
        } else {
            std::cerr << "Failed to retrieve measure: " << err << std::endl;
            exit(EXIT_FAILURE);
        }
    } else {
        std::cerr << "Failed to grab: " << err << std::endl;
        exit(EXIT_FAILURE);
    }
}

std::string getTimestampStr() {
    time_t time = std::time(nullptr);
    tm tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S");
    return oss.str();
}

void handler(int s) {
    // close files and shtuff
    stop = 1;
}

int main(int argc, char** argv) {
    fs::path dirName = fs::current_path().parent_path() / "recordings";
    if (!fs::is_directory(dirName)) {
        fs::create_directory(dirName);
    }
    sl::Camera zed;
    // Set configuration parameters
    // TODO confirm correct
    sl::InitParameters init_params;
    init_params.sdk_verbose = true;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 15;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
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
    while (!stop) {
        get_pcl_cloud(p_pcl_cloud, zed);
//
        sl::SensorsData sensorData;
        zed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

        std::string timeStamp = getTimestampStr();
        fs::path fileName = dirName / (timeStamp + " #" + std::to_string(++frame) + ".pcd");
        writer.writeBinaryCompressed(fileName, *p_pcl_cloud);
        std::ifstream pcdFile(fileName);
        std::stringstream pcdStream;
        pcdStream << "TIMESTAMP " << timeStamp << '\n'
                  << "MAGNETOMETER_STATE " << sensorData.magnetometer.magnetic_heading_state << '\n'
                  << "MAGNETOMETER_HEADING " << sensorData.magnetometer.magnetic_heading << '\n'
                  << "IMU_ANG_VEL_X " << sensorData.imu.angular_velocity.x << '\n'
                  << "IMU_ANG_VEL_Y " << sensorData.imu.angular_velocity.y << '\n'
                  << "IMU_ANG_VEL_Z " << sensorData.imu.angular_velocity.z << '\n'
                  << "IMU_LIN_ACCEL_X " << sensorData.imu.linear_acceleration.x << '\n'
                  << "IMU_LIN_ACCEL_Y " << sensorData.imu.linear_acceleration.y << '\n'
                  << "IMU_LIN_ACCEL_Z " << sensorData.imu.linear_acceleration.z << '\n'
                  << "BAROMETER_PRESSURE " << sensorData.barometer.pressure << '\n'
                  << "TEMP " << sensorData.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT] << '\n';
        pcdStream << pcdFile.rdbuf();
    }

    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}