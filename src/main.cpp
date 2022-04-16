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

void writeCloud(ofstream &fout, GPU_Cloud gpu_pc, int width, int height) 
{
    if (!fout.is_open()) cerr << "Could not open file!" << endl;

    glm::vec4* pc = new glm::vec4[gpu_pc.size];
    cudaMemcpy(pc, gpu_pc.data, sizeof(glm::vec4) * gpu_pc.size, cudaMemcpyDeviceToHost);

    unsigned int rgba;
    for (int i = 0; i < gpu_pc.size; ++i) {
        const vec4& point = pc[i];
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
    // Get camera information (ZED serial number)
<<<<<<< HEAD
    //int zed_serial = zed.getCameraInformation().serial_number;
    //printf("Hello! This is my serial number: %d\n", zed_serial);
=======
    sl::Mat pcdMat;
    GPU_Cloud gpuCloud;
    zed.retrieveMeasure(pcdMat, sl::MEASURE::XYZRGBA, sl::MEM::GPU);
    getRawCloud(gpuCloud, pcdMat);
>>>>>>> 6020464cadbb7463944b541c9c9ee9ed30c48e5e

    // Get camera resolution
    int width = 69;
    int height = 69; 

    sl::Mat pcdMat;
    GPU_Cloud gpu_pc;
    zed.retrieveMeasure(pcdMat, sl::MEASURE::XYZRGBA, sl::MEM::GPU);
    getRawCloud(gpu_pc, frame);

    ofstream fout("test.pcd");
    writeCloud(fout, gpu_pc, width, height); 

    // Close the camera
    zed.close();
    return 0;
}