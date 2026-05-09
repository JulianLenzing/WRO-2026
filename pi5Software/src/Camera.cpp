#include "Camera.h"
#include <stdexcept>

Camera::Camera()
{
	int width = 640;
	int height = 480;
    // GStreamer pipeline using libcamera
	std::string pipeline =
		"libcamerasrc awb-enable=true awb-mode=auto ae-enable=true ae-constraint-mode=shadows ae-metering-mode=matrix ae-exposure-mode=long ! "// analogue-gain-mode=manual analogue-gain=1 ! "// exposure-time-mode=manual exposure-time=1000000000 ! "
		"video/x-raw,format=NV12,width=1296,height=972,framerate=30/1 ! "
		"videoconvert ! "
		"videoscale ! "
		"video/x-raw,width=" + std::to_string(width) +
		",height=" + std::to_string(height) +
		",format=BGR ! "
		"videoflip method=rotate-180 ! "
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

    // Draw a black rectangle to remove unwanted color from outside the parcourse 
    frame(cv::Rect(0, 0, frame.cols, 23)) = cv::Scalar(0, 0, 0);

    return frame;
}

bool Camera::isOpened() const
{
    return cap.isOpened();
}
