#include <iostream>
#include <ostream>
#include <fstream>
#include <iomanip>


#include "RobotSystem.h"
#include "StateMachine.h"
#include "State.h"
#include "lidar.h"
#include "poseEstimation.h"
#include "LidarPoint.h"
#include "Vec2f.h"
#include "Graphics.h"
#include "DisplayData.h"
#include "DisplayUserInterface.h"
#include "GuidanceData.h"
#include "guidance.h"
#include "InitState.h"
#include "StartState.h"
#include "RunCourseState.h"
#include "StopState.h"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

static int initGLFW() {
    // Set error callback (optional)
    glfwSetErrorCallback(glfw_error_callback);
    // Initialize GLFW
    if (!glfwInit()) {
        return 0;
    }
    return 1;
}

/*
enum MainState{
	MainStateInit, 					// Setup software
	MainStateWaitForStart, 				// For the start button to be pressed
	MainStateStart, 				// Start Sensors
	MainStateFindStartingLocation, 			// Find which in which of the possible locations the robot is
	MainStateUnpark, 				// Leave the parking area
	MainStateRunCourse,				// Complete the rounds and pass obstacles on the correct side
	MainStatePark,					// Leave the vehicle inside the parking area
	MainStateEnd,					// Stop sensors
	MainStateDeinit					// Free resources and stop software
};
*/

int main(){
	// Initialize GLFW
	if (!initGLFW()) {std::cout<<"Could not initialise GLFW"<<std::endl; return 0;}
    
	// Set OpenGL version hints
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	    
	RobotSystem robot;
	StateMachine sm;
	InitState initState;
	StartState startState;
	RunCourseState runCourseState;
	StopState stopState;
	//std::thread guidanceThread(guidanceMain, ref(robot.guidanceData));

	sm.setState(&initState, robot);
	sm.setState(&startState, robot);
	while(!robot.startActivated)
	{
		sm.update(robot);
	}
	
	sm.setState(&runCourseState, robot);
	while(true){
		sm.update(robot);
		if(robot.displayUI.exit) break;
		std::this_thread::sleep_for(std::chrono::milliseconds(110));
	}

	sm.setState(&stopState, robot);

	// Stop other threads
	//guidanceData.terminate();
	//if (guidanceThread.joinable()) {
	//	guidanceThread.join();
	//}
	return 0;
}
