#include <sl/Camera.hpp>
#include <glm/vec4.hpp>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "json.hpp"

void writeCloud(std::ofstream &fout, GPU_Cloud gpu_pc, int width, int height) {
    if (!fout.is_open()) std::cerr << "Could not open file!" << std::endl;

    glm::vec4* pc = new glm::vec4[gpu_pc.size];
    cudaMemcpy(pc, gpu_pc.data, sizeof(glm::vec4) * gpu_pc.size, cudaMemcpyDeviceToHost);

    unsigned int rgba;
    for (int i = 0; i < gpu_pc.size; ++i) {
        const glm::vec4& point = pc[i];
        rgba = *reinterpret_cast<const unsigned int*>(&point.w);
        rgba = (rgba & 0xFF) << 16 | (rgba & 0xFF00) | (rgba & 0xFF0000) >> 16;

        fout << std::setprecision(8) << point.x << ' ' << point.y << ' ' << point.z << ' ' << rgba << '\n';
    }

    std::cout << "Wrote a point cloud of size " << width << " x " << height << '\n';

    delete[] pc;
}



int main(int argc, char **argv) {

    // Create a ZED camera object
    sl::Camera zed;

    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.sdk_verbose = true; // Disable verbose mode

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        exit(-1);
    }

    sl::Resolution cloud_res = zed.getCameraInformation().camera_configuration.resolution;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_res.area());
    

    // Get camera information (ZED serial number)
    sl::Mat pcdMat;
    zed.retrieveMeasure(pcdMat, sl::MEASURE::XYZRGBA, sl::MEM::GPU);

    float *p_data_cloud = pcdMat.getPtr<float>();
    int index = 0;

    // Check and adjust points for PCL format
    for (auto &it : p_pcl_point_cloud->points) {
        float X = p_data_cloud[index];
        if (!isValidMeasure(X)) // Checking if it's a valid point
            it.x = it.y = it.z = it.rgb = 0;
        else {
            it.x = X;
            it.y = p_data_cloud[index + 1];
            it.z = p_data_cloud[index + 2];
            it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
        }
        index += 4;
    }

    pcl::PCDWriter writer;
    writer.writeBinaryCompressed();

    // Get camera resolution
    int width = 69;
    int height = 69; 

    std::ofstream fout("test.pcd");
    writeCloud(fout, gpuCloud, width, height); 

    // Close the camera
    zed.close();
    return 0;
}