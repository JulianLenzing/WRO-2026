/* std includes */
#include <cmath>
#include <iostream>
#include <optional>

/* user includes */
#include <lgpio.h>
#include "GuidanceData.h"
#include "PwmController.h"

/* Steering servo and motor parameters */
#define MAX_STEERING_ANGLE 45
#define SERVO_DUTY_CYCLE_RANGE 1.0f
#define MOTOR_DUTY_CYCLE_RANGE 1.0f

using namespace std;

static float normaliseAngle(float angle) {
    angle = fmodf(fmodf(angle, 2*M_PI) + 2*M_PI, 2*M_PI);
    return angle;
}

static float correctSteering(float direction, float heading) {
    float steeringAngle = direction - heading;
    steeringAngle = normaliseAngle(steeringAngle);
    return steeringAngle;
}

void guidanceMain(GuidanceData& guidanceData)
{
    ServoController steering(0, MAX_STEERING_ANGLE, SERVO_DUTY_CYCLE_RANGE);
	MotorController motor(1, MOTOR_DUTY_CYCLE_RANGE);
    motor.unlockControl();
    optional<Vec2f> currentWaypoint;
    currentWaypoint.reset();
    bool atWaypoint = false;

    while (guidanceData.getThreadStatus())
    {        
        if(guidanceData.getGuidanceStatus()) {
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

                if(0) currentWaypoint.reset(); // Placeholder for when we have a way to determine if we are at the waypoint
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    steering.setMiddle();
    motor.setSpeed(0);
}

