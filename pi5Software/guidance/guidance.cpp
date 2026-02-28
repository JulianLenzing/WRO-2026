/* std includes */
#include <cmath>
#include <iostream>
#include <optional>

/* user includes */
#include <lgpio.h>
#include "GuidanceData.h"
#include "servoControl.h"

/* Steering Servo parameters */
#define MAX_STEERING_ANGLE 45

using namespace std;

float normaliseAngle(float angle) {
    angle = fmodf(fmodf(angle, 2*M_PI) + 2*M_PI, 2*M_PI);
    return angle;
}

float correctSteering(float direction, float heading) {
    float steeringAngle = direction - heading;
    steeringAngle = normaliseAngle(steeringAngle);
}

void guidanceMain(GuidanceData& guidanceData)
{
    ServoControl sc(MAX_STEERING_ANGLE, 1.0f);
    while (guidanceData.getThreadStatus())
    {
        if(guidanceData.getGuidanceStatus()) {
            optional<Vec2f> optWaypoint = guidanceData.getWaypoint();
            if (optWaypoint.has_value())
            {
                Vec2f waypoint = optWaypoint.value();
                int count = guidanceData.getWaypointCount();
                printf("Count: %d X: %.2f Y: %.2f\n", count, waypoint.x, waypoint.y);
            }
            //sc.setAngle(3.1415);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

