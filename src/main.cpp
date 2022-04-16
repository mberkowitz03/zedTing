#include <sl/Camera.hpp>
#include <glm/vec4.hpp>

struct GPU_Cloud {
    sl::float4* data;
    int size;
};

void getRawCloud(GPU_Cloud& pc, sl::Mat& zed_cloud) {
    sl::float4* ptr = zed_cloud.getPtr<sl::float4>(sl::MEM::GPU);
    pc.data = (sl::float4*) ptr;
    pc.size = zed_cloud.getWidth() * zed_cloud.getHeight();
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
    // Get camera information (ZED serial number)
    sl::Mat pcdMat;
    GPU_Cloud gpuCloud;
    zed.retrieveMeasure(pcdMat, sl::MEASURE::XYZRGBA, sl::MEM::GPU);
    getRawCloud(gpuCloud, pcdMat);



    //int zed_serial = zed.getCameraInformation().serial_number;
    //printf("Hello! This is my serial number: %d\n", zed_serial);

    // Close the camera
    zed.close();
    return 0;
}