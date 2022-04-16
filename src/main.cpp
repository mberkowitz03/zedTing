#include <sl/Camera.hpp>
#include <glm/vec4.hpp>

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
    
    //Zed point cloud 
    sl::Mat &zed_pc

    //PCL point cloud
    sl::Resolution cloud_res(640, 360);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                p_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_res.area());

    //Data stream 
    std::ostringstream oss;

    get_pcl_cloud(p_pcl_cloud, zed_pc); 
    write_binary_data(oss, *p_pcl_cloud);

    // Close the camera
    zed.close();
    return 0;
}

