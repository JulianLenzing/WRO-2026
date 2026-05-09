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
#define REVERSE_MAX_THROTTLE 1.0f
#define REVERSE_MIN_THROTTLE 0.35f
#define ACCELERATION_CONSTANT 0.5f // The distance from waypoint where full throtlle is reached in meters

/* Guidance parameters */
#define WAYPOINT_THRESHOLD 0.05f
#define PASSED_WAYPOINT_THRESHOLD 0.15f
#define MAX_CLOSING_ANGLE (M_PI/2.0f)

using namespace std;

void guidanceMain(GuidanceData& guidanceData)
{
    ServoController steering(0, MAX_STEERING_ANGLE, SERVO_DUTY_CYCLE_RANGE);
	MotorController motor(1, MOTOR_DUTY_CYCLE_RANGE);
    steering.invert();
    steering.setMiddle();

    optional<Waypoint> currentWaypoint;
    float heading = 0;
    Vec2f position(0, 0);

    #ifdef SIMULATION
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    #endif

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
                Vec2f waypointDirectionVector = Vec2f(cosf(currentWaypoint.value().heading), sinf(currentWaypoint.value().heading));
                Line waypointLine(currentWaypoint.value().point, currentWaypoint.value().point + waypointDirectionVector);
                Vec2f closestPointOnLine = waypointLine.closestPointOnInfinite(position);
                Vec2f relativePosition = position - closestPointOnLine;
                float lineDistance = relativePosition.length();

                float direction = currentWaypoint.value().heading;
                float closingDistance = currentWaypoint.value().closingDistance;
                float exp = 1.0f;
                    
                float closingAngle = powf(clamp(lineDistance, 0.0f, closingDistance) / closingDistance, exp) * (M_PI/2.0f);

                if (relativePosition.dot(waypointLine.normal()) > 0.0f) closingAngle = -closingAngle;
                if (currentWaypoint.value().reverse) closingAngle = -closingAngle;
                direction += closingAngle;
                if (currentWaypoint.value().reverse) direction += M_PI;

                float steeringAngle;
                if (!currentWaypoint.value().reverse) steeringAngle = direction-heading;
                else steeringAngle = fmodf(heading+M_PI, 2.0f*M_PI) - direction;
                steering.setAngle(steeringAngle);

                // Throttle is proportional to distance but capped at MAX_THROTTLE and at minimum MIN_THROTTLE to ensure the robot keeps moving
                float maxThrottle;
                float minThrottle;
                if(!currentWaypoint.value().reverse){
                    maxThrottle = MAX_THROTTLE;
                    minThrottle = MIN_THROTTLE;
                }
                else {
                    maxThrottle = REVERSE_MAX_THROTTLE;
                    minThrottle = REVERSE_MIN_THROTTLE;
                }
                
                float throttle = maxThrottle;
                if (currentWaypoint.value().slow) throttle = clamp(distance / ACCELERATION_CONSTANT * (maxThrottle - minThrottle), minThrottle, maxThrottle);

                // Throttle is capped at maxThrottle and minThrottle and is reduced at large steering angles and when closing to a waypoint with slow set
                float currentAbsoluteSteeringAngle = steering.getAngle();
                if(currentAbsoluteSteeringAngle > M_PI) currentAbsoluteSteeringAngle = 2*M_PI - currentAbsoluteSteeringAngle;
                throttle = clamp(float(throttle - (currentAbsoluteSteeringAngle / MAX_STEERING_ANGLE * (maxThrottle - minThrottle))), minThrottle, maxThrottle);

                // Enforce max throttle from waypoint
                if (throttle > currentWaypoint.value().maxThrottle)
                {
                    throttle = currentWaypoint.value().maxThrottle;
                    if (currentWaypoint.value().maxThrottle < MIN_THROTTLE && !currentWaypoint.value().reverse || currentWaypoint.value().maxThrottle < REVERSE_MIN_THROTTLE && currentWaypoint.value().reverse)
                    {
                        if(!currentWaypoint.value().reverse) throttle = MIN_THROTTLE;
                        else throttle = REVERSE_MIN_THROTTLE;
                    } 
                }

                if (currentWaypoint.value().reverse) throttle = -throttle;
                motor.setThrottle(throttle);

                // Send values to UI
                guidanceData.setUiData(steering.getAngle(), motor.getThrottle());

                // Check if we are at the waypoint and if so reset the current waypoint to get the next one from the queue
                Vec2f relativePositionToWp = position - currentWaypoint.value().point;
                if(
                    (relativePositionToWp.dot(waypointDirectionVector) > 0.0f && !currentWaypoint.value().reverse)
                    || (relativePositionToWp.dot(waypointDirectionVector) < 0.0f && currentWaypoint.value().reverse)
                    ) currentWaypoint.reset();
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


