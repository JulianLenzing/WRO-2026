/* std includes */
#include <cmath>
#include <iostream>
#include <optional>

/* user includes */
#include <lgpio.h>
#include "GuidanceData.h"
#include "PwmController.h"

/* Steering servo and motor parameters */
#define MAX_STEERING_ANGLE (M_PI/2.0f) 
#define SERVO_DUTY_CYCLE_RANGE 1.0f
#define MOTOR_DUTY_CYCLE_RANGE 1.0f
#define MAX_THROTTLE 0.4f
#define MIN_THROTTLE 0.1f
#define ACCELERATION_CONSTANT 4.0f

/* Guidance parameters */
#define WAYPOINT_THRESHOLD 0.1f

using namespace std;

void guidanceMain(GuidanceData& guidanceData)
{
    ServoController steering(0, MAX_STEERING_ANGLE, SERVO_DUTY_CYCLE_RANGE);
	MotorController motor(1, MOTOR_DUTY_CYCLE_RANGE);
    steering.invert();
    steering.setMiddle();
    motor.unlockControl();

    optional<Vec2f> currentWaypoint;
    currentWaypoint.reset();
    bool atWaypoint = false;
    float heading = 0;
    Vec2f position(0, 0);

    while (guidanceData.getThreadStatus())
    {        
        if(guidanceData.getGuidanceStatus()) {
            // Get data from main thread
            guidanceData.getRobotData(position, heading);

            // Check if we dont have a current waypoint and if a new one is available and try to get one from the queue
            if (guidanceData.getWaypointCount() > 0 && !currentWaypoint.has_value())
            {
                optional<Vec2f> optWaypoint = guidanceData.getWaypoint();
                if (optWaypoint.has_value())
                {
                    currentWaypoint = optWaypoint.value();
                    cout << "Steering to new Waypoint: (" << currentWaypoint->x << ", " << currentWaypoint->y << ")" << endl;
                }
            }

            // If we have a waypoint, calculate the steering angle and set the servo and motor accordingly
            if(currentWaypoint.has_value()) {
                // Get distance to waypoint
                float distance = Vec2f(currentWaypoint.value() - position).length();

                // Set steering and throttle
                float direction = atan2f(currentWaypoint.value().y - position.y, currentWaypoint.value().x - position.x);
                steering.setAngle(direction-heading);

                // Throttle is proportional to distance but capped at MAX_THROTTLE and at minimum MIN_THROTTLE to ensure the robot keeps moving
                float throttle = clamp(distance * ACCELERATION_CONSTANT, MIN_THROTTLE, MAX_THROTTLE);
                motor.setThrottle(throttle);

                // Send values to UI
                guidanceData.setUiData(steering.getAngle(), motor.getThrottle());

                // Check if we are at the waypoint and if so reset the current waypoint to get the next one from the queue
                if(distance < WAYPOINT_THRESHOLD) currentWaypoint.reset(); 
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    steering.setMiddle();
    motor.setThrottle(0);
}

