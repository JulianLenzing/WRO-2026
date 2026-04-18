#include "Camera.h"
#include <fstream>
#include <vector>
#include <string>

std::string cam_path = "/dev/shm/my_godot_ipc_shm.raw";
int width = 640;
int height = 480;
size_t cam_frame_size;
std::vector<uint8_t> cam_buffer;

    Camera::Camera()
    {
        cam_frame_size = width * height * 4; // RGBA
        cam_buffer.resize(cam_frame_size);
    }

    Camera::~Camera() = default;

    cv::Mat Camera::grabFrame()
    {
        std::ifstream file(cam_path, std::ios::binary);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open camera device/file");
        }

        file.seekg(0);  // rewind before every read
        file.read(reinterpret_cast<char*>(cam_buffer.data()), cam_frame_size);

        if (file.gcount() != cam_frame_size)
        {
            throw std::runtime_error("Incomplete frame read");
        }

        cv::Mat frame(height, width, CV_8UC4, cam_buffer.data());
        cv::Mat display;
        cv::cvtColor(frame, display, cv::COLOR_RGBA2BGR);

        return display.clone();
    }
