/* std includes */
#include <cmath>
#include <iostream>
#include <optional>
#include <queue>

/* user includes */
#include "GuidanceData.h"
#include "PwmController.h"
#include "../include/Waypoint.h"

#include "Graphics.h"
#include "DisplayData.h"

/* Steering servo and motor parameters */
#define MAX_STEERING_ANGLE (M_PI/2.0f) 
#define SERVO_DUTY_CYCLE_RANGE 1.0f
#define MOTOR_DUTY_CYCLE_RANGE 1.0f
#define MAX_THROTTLE 1.0f
#define MIN_THROTTLE 0.25f
#define ACCELERATION_CONSTANT 0.5f // The distance from waypoint where full throtlle is reached in meters

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

    optional<Waypoint> currentWaypoint;
    queue<Vec2f> steeringPointBuffer;
    bool atWaypoint = false;
    float heading = 0;
    Vec2f position(0, 0);

    while (guidanceData.getThreadStatus())
    {        
        if(guidanceData.getGuidanceStatus()) {
            // Get data from main thread
            guidanceData.getRobotData(position, heading);

            // Check if we dont have a current waypoint and if a new one is available and try to get one from the queue
            if (!currentWaypoint.has_value())
            {
                if (guidanceData.getWaypointCount() > 0)
                {
                    currentWaypoint = guidanceData.getWaypoint();
                    guidanceData.setReachedLastWaypoint(false);
                }
                else guidanceData.setReachedLastWaypoint(true);
            }

            // If we have a waypoint, calculate the steering angle and set the servo and motor accordingly
            if(currentWaypoint.has_value()) {
                // Get distance to waypoint
                float distance = Vec2f(currentWaypoint.value().point - position).length();

                // Set steering and throttle
                float direction = atan2f(currentWaypoint.value().point.y - position.y, currentWaypoint.value().point.x - position.x);
                steering.setAngle(direction-heading);

                // Throttle is proportional to distance but capped at MAX_THROTTLE and at minimum MIN_THROTTLE to ensure the robot keeps moving
                float throttle = MAX_THROTTLE;
                if (currentWaypoint.value().slow) throttle = clamp(distance / ACCELERATION_CONSTANT * (MAX_THROTTLE - MIN_THROTTLE), MIN_THROTTLE, MAX_THROTTLE);

                // Throttle is capped at MAX_THROTTLE and MIN_THROTTLE and is reduced at large steering angles
                float currentAbsoluteSteeringAngle = steering.getAngle();
                if(currentAbsoluteSteeringAngle > M_PI) currentAbsoluteSteeringAngle = 2*M_PI - currentAbsoluteSteeringAngle;
                throttle = clamp(float(throttle - (currentAbsoluteSteeringAngle / MAX_STEERING_ANGLE * (MAX_THROTTLE - MIN_THROTTLE))), MIN_THROTTLE, MAX_THROTTLE);

                motor.setThrottle(throttle);
                //printf("Distance %.2f Throttle %.2f\n", distance, throttle);

                // Send values to UI
                guidanceData.setUiData(steering.getAngle(), motor.getThrottle());

                // Check if we are at the waypoint and if so reset the current waypoint to get the next one from the queue
                if(distance < WAYPOINT_THRESHOLD) currentWaypoint.reset();
            }
            else {
                /* Lock steering and stop motor if no current waypoint is set*/
                steering.setMiddle();
                motor.setThrottle(0.0f);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    steering.setMiddle();
    motor.setThrottle(0.0f);
}


