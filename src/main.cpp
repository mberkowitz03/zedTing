#include <sl/Camera.hpp>
#include <glm/vec4.hpp>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "json.hpp"

<<<<<<< HEAD
void get_pcl_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_cloud, sl::Mat &zed_pc)
{
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
=======
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
>>>>>>> 1075c0d3d032975a706f9ac04851289ee3eb4b06
    }
}

void write_binary_data(std::ostream &os, pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud)
{   
    pcl::PCDWriter::writeBinaryCompressed(os, pcl_cloud); 
}



int main(int argc, char **argv) {

    // Create a ZED camera object
    sl::Camera zed;

    // Set configuration parameters (@TODO: confirm)
    sl::InitParameters init_params;
    init_params.sdk_verbose = true; // Disable verbose mode
    init_params.camera_resolution = RESOLUTION::VGA;
    init_params.camera_fps = 30;
    init_params.coordinate_units = UNIT::MILLIMETER;
    init_params.depth_mode = DEPTH_MODE::ULTRA;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        exit(-1);
    }
<<<<<<< HEAD
    
    //Zed point cloud 
    sl::Mat &zed_pc
=======

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
>>>>>>> 1075c0d3d032975a706f9ac04851289ee3eb4b06

    //PCL point cloud
    sl::Resolution cloud_res(640, 360);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                p_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_res.area());

<<<<<<< HEAD
    //Data stream 
    std::ostringstream oss;

    get_pcl_cloud(p_pcl_cloud, zed_pc); 
    write_binary_data(oss, *p_pcl_cloud);
=======
    std::ofstream fout("test.pcd");
    writeCloud(fout, gpuCloud, width, height); 
>>>>>>> 1075c0d3d032975a706f9ac04851289ee3eb4b06

    // Close the camera
    zed.close();
    return 0;
}

