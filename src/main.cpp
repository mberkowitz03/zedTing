#include <sl/Camera.hpp>
#include <glm/vec4.hpp>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "json.hpp"

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
        if (!isValidMeasure(X)) // Checking if it's a valid point
            it.x = it.y = it.z = it.rgb = 0;
        else {
            it.x = X;
            it.y = p_zed_pc[index + 1];
            it.z = p_zed_pc[index + 2];
            it.rgb = convertColor(p_zed_pc[index + 3]); // Convert a 32bits float into a pcl .rgb format
        }
        index += 4;
    }
}

void write_binary_data(std::string fileName, pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud)
{   
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(fileName, pcl_cloud); 
}



int main(int argc, char **argv) {

    // Create a ZED camera object
    sl::Camera zed;

    // Set configuration parameters (@TODO: confirm)
    sl::InitParameters init_params;
    init_params.sdk_verbose = true; // Disable verbose mode
    init_params.camera_resolution = sl::RESOLUTION::VGA;
    init_params.camera_fps = 30;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        exit(-1);
    }

    //PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                p_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //Data stream 
    std::string fileName = "test.pcd";

    get_pcl_cloud(p_pcl_cloud, zed); 
    write_binary_data(fileName, *p_pcl_cloud);

    // Close the camera
    zed.close();
    return 0;
}

