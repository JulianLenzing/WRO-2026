#include "Camera.h"
#include <stdexcept>

Camera::Camera()
{
	int width = 640;
	int height = 480;
    // GStreamer pipeline using libcamera
	std::string pipeline =
		"libcamerasrc ! "
		"video/x-raw,format=NV12,width=1296,height=972,framerate=30/1 ! "
		"videoconvert ! "
		"videoscale ! "
		"video/x-raw,width=" + std::to_string(width) +
		",height=" + std::to_string(height) +
		",format=BGR ! "
		"appsink drop=true max-buffers=1 sync=false";

    cap.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        throw std::runtime_error("Failed to open camera via GStreamer");
    }
}

Camera::~Camera()
{
    if (cap.isOpened()) {
        cap.release();
    }
}

cv::Mat Camera::grabFrame()
{
    cv::Mat frame;
    cap >> frame;   // grab + decode

    if (frame.empty()) {
        return cv::Mat();
    }

    return frame;
}

bool Camera::isOpened() const
{
    return cap.isOpened();
}
