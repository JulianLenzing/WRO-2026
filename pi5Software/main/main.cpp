#include <iostream>
#include <ostream>

#include "lidar.h"
#include "poseEstimation.h"
#include "LidarPoint.h"
#include "Vec2f.h"
#include "Graphics.h"
#include "DisplayData.h"
#include "DisplayUserInterface.h"
#include "GuidanceData.h"
#include "guidance.h"


#include <fstream>
#include <iomanip>

void writeLidarScanToFile(const LidarScan& scan, const std::string& filename)
{
    static std::ofstream out(filename, std::ios::app); // append mode

    if (!out.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (const auto& p : scan.scan) {
        out << std::fixed << std::setprecision(2)
            << p.distance << " "
            << p.angle << "\n";
    }

    out << "----\n"; // separator between scans (optional)
}


static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

static int initGLFW() {
    // Set error callback (optional)
    glfwSetErrorCallback(glfw_error_callback);
    // Initialize GLFW
    if (!glfwInit()) {
        std::cout << "Failed to initialize GLFW\n";
        return 0;
    }
    return 1;
}

int main(){
	// Initialize GLFW
    if (!initGLFW()) {std::cout<<"Could not initialise GLFW"<<std::endl; return 0;}
    // Set OpenGL version hints
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	
	// Initialize Guis
	Graphics gp(1000, 1000, BLACK);
	Visibility visibility;
	DisplayUserInterface displayUI(visibility);
	visibility.setLineVisibility(SLAM_DEBUG_LINE, false);
	
	// Init pose estimation
	initPoseEstimation();
	
	// Init the Lidar
	sl::ILidarDriver* lidarDriver = *sl::createLidarDriver();
	if(!lidarDriver) printf("Lidar Driver not correctly initialised!");
	
	// Init guidance
	GuidanceData guidanceData;
    //std::thread guidanceThread(guidanceMain, ref(guidanceData));
    
    // --------------------- Start ------------------------ //
    
    // Start Lidar
    startLidar(lidarDriver);
    
    // Start guidance
    //guidanceData.start();
	
	Vec2f estimatedPosition(0.5f, 0.5f);
	while(true){
		dpd.clear();
		LidarScan lidarScan;
		getLidarScan(lidarDriver, lidarScan, 1, 0.25);
		//for(auto p : lidarScan.scan) {printf("%.2f %.2f\n", p.distance, p.angle);}
		writeLidarScanToFile(lidarScan, "LidarTestData");
		Vec2f newEstimatedPosition(estimatedPosition);
		doPoseEstimation(lidarScan, estimatedPosition, 0.1f, newEstimatedPosition);
		estimatedPosition = estimatedPosition * 0.8 + newEstimatedPosition * 0.2;
		guidanceData.appendWaypoint(Vec2f(1, 1));
		
		displayUI.update();
		dpd.updateVisibility(visibility);	
		gp.update(dpd);
		if(displayUI.exit) break;
	
		std::this_thread::sleep_for(std::chrono::milliseconds(110));
	}

	// Stop sensors
	stopLidar(lidarDriver);
	
	// Stop other threads
	guidanceData.terminate();
	//if (guidanceThread.joinable()) {
	//	guidanceThread.join();
	//}
	return 0;
}
