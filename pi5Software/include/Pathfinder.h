#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <optional>
#include <iostream>

#include "Vec2f.h"
#include "Waypoint.h"
#include "Obstacle.h"
#include "GuidanceData.h"

#define ROUNDS_TO_DRIVE 3
#define WAYPOINT_INTERPOLATION_COUNT 15

enum RUN_DIRECTION
{
    RUN_DIRECTION_CCW = 0, // Counterclockwise
    RUN_DIRECTION_CW = 1, // Clockwise
};

enum PATHFINDER_STATE
{
    PATHFINDER_STATE_INITIAL,
    PATHFINDER_STATE_SIDES,
    PATHFINDER_STATE_FINAL,
    PATHFINDER_STATE_STOP
};

class Path
{
public:
    Path() : waypoints(), name("Default path name") {}
    Path(std::string pName, std::vector<Waypoint> pWaypoints = {}) : name(pName), waypoints(pWaypoints) {}
    ~Path() {}

    std::vector<Waypoint> waypoints;
    std::string name;
};

class Side
{
public:
    Side(Vec2f pBasePosition, float pDirection, Vec2f pLowerLeft, Vec2f pUpperRight, Vec2f pMiddle) : basePosition(pBasePosition), direction(pDirection), middle(pMiddle), hasPath(false), lowerLeft(pLowerLeft), upperRight(pUpperRight) {}
    Side() : basePosition(Vec2f(0.0f, 0.0f)), direction(0.0f), lowerLeft(Vec2f(0.0f, 0.0f)),
    upperRight(Vec2f(0.0f, 0.0f)), middle(Vec2f(0.0f, 0.0f)), hasPath(false) {}
    ~Side() {}

    Side& operator=(const Side& other)
    {
        if (this == &other)
            return *this; // handle self-assignment

        basePosition = other.basePosition;
        direction    = other.direction;
        middle       = other.middle;
        hasPath      = other.hasPath;
        lowerLeft    = other.lowerLeft;
        upperRight   = other.upperRight;

        return *this;
    }

    void copyPath(const Path& otherPath)
    {
        hasPath = true;
        float deltaDirection = direction;
        Vec2f deltaBasePosition = basePosition;

        path = otherPath; // copy name + waypoints

        // Rotate by deltaDirection
        float cosA = cos(deltaDirection);
        float sinA = sin(deltaDirection);

        for (Waypoint& wp : path.waypoints)
        {
            // Translate relative to other's base
            Vec2f relative = wp.point;

            Vec2f rotated(
                relative.x * cosA - relative.y * sinA,
                relative.x * sinA + relative.y * cosA
            );

            // Translate to this side's base
            wp.point = rotated + basePosition;
            wp.heading += deltaDirection;
        }
    }

    void mirrorPath()
    {
        for (Waypoint& wp : path.waypoints)
        {
            Vec2f& p = wp.point;
            Vec2f d(cosf(direction), sinf(direction));
            Vec2f v = p - middle;

            float projection = v.x * d.x + v.y * d.y;
            Vec2f v2 = d * projection;
            Vec2f v3 = p - v2 * 2;
            wp.point = v3;
        }
    }

    Vec2f basePosition;
    float direction;
    Vec2f lowerLeft;
    Vec2f upperRight;
    Vec2f middle;
    Path path;
    bool hasPath;
};

class Pathfinder
{
public:
    Pathfinder() : currentSideIndex(0), runDirection(RUN_DIRECTION_CCW), round(1), pathfinderState(PATHFINDER_STATE_INITIAL), stop(false)
    {
        sides.emplace_back(Vec2f(0.0f, 0.0f), float(0.0f), Vec2f(0.9f, 0.0f), Vec2f(2.1f, 1.0f), Vec2f(1.5f, 0.5f));
        sides.emplace_back(Vec2f(3.0f, 0.0f), float(M_PI/2.0f), Vec2f(2.0f, 0.9f), Vec2f(3.0f, 2.1f), Vec2f(2.5f, 1.5f));
        sides.emplace_back(Vec2f(3.0f, 3.0f), float(M_PI), Vec2f(0.9f, 2.0f), Vec2f(2.1f, 3.0f), Vec2f(1.5f, 2.5f));
        sides.emplace_back(Vec2f(0.0f, 3.0f), float(3.0f*M_PI/2.0f), Vec2f(0.0f, 0.9f), Vec2f(1.0f, 2.1f), Vec2f(0.5f, 1.5f));

        initialSide = sides[0];
        finalSide = sides[0];

        initPaths();
    }

    void setRunDirection(enum RUN_DIRECTION pRunDirection)
    {
        runDirection = pRunDirection;
    }

    void update(Vec2f position, float heading, std::vector<Obstacle> obstacles, GuidanceData& guidanceData)
    {
        switch (pathfinderState)
        {
            case PATHFINDER_STATE_INITIAL:
                initialSide.copyPath(initial);
                if (runDirection == RUN_DIRECTION_CW) initialSide.mirrorPath();

                // Append the waypoints of the chosen path and correct for run direction
                for (const Waypoint& wp : initialSide.path.waypoints)
                {
                    guidanceData.appendWaypoint(wp);
                }

                // Index to next side
                if (runDirection == RUN_DIRECTION_CCW) currentSideIndex++;
                else currentSideIndex--;
                currentSideIndex = (currentSideIndex + 4) % 4;
                pathfinderState = PATHFINDER_STATE_SIDES;
                //printf("Init Round: %d Index: %d\n", round, currentSideIndex);
                break;

            case PATHFINDER_STATE_SIDES:
                //printf("Current Index: %d\n", currentSideIndex);
                if (!sides[currentSideIndex].hasPath)
                {
                    // If the current side does not have a path one must be determined
                    std::vector<Obstacle> sideObstacles;
                    for (const Obstacle& obs : obstacles)
                    {
                        if (inBox(obs.position, sides[currentSideIndex].lowerLeft, sides[currentSideIndex].upperRight))
                        {
                            sideObstacles.push_back(obs);
                        }
                    }
                    //printf("Number of obstacles in side: %d\n", sideObstacles.size());
                    if (sideObstacles.size() <= 0) return; // Wait until an obstacle in the current side was detected
                    Obstacle obstacle = sideObstacles[0];
                    for (const Obstacle& obs : sideObstacles)
                    {
                        if (obs.count > obstacle.count) obstacle = obs;
                    }
                    if (obstacle.getColor() == OBSTACLE_COLOUR_UNKNOWN) return;
                    //printf("Obstacle position number: %d\n", obstacle.positionNumber);

                    sides[currentSideIndex].copyPath(getPathFromObstacle(obstacle));
                    if (runDirection == RUN_DIRECTION_CW) sides[currentSideIndex].mirrorPath();
                }

                // Append the waypoints of the chosen path and correct for run direction
                for (const Waypoint& wp : sides[currentSideIndex].path.waypoints)
                {
                    guidanceData.appendWaypoint(wp);
                }

                // Index to next side
                if (runDirection == RUN_DIRECTION_CCW) currentSideIndex++;
                else currentSideIndex--;
                currentSideIndex = (currentSideIndex + 4) % 4;
                if (currentSideIndex == 0) round++;

                //printf("Round: %d Index: %d\n", round, currentSideIndex);

                if (round == ROUNDS_TO_DRIVE + 1)
                {
                    pathfinderState = PATHFINDER_STATE_FINAL;
                    round = ROUNDS_TO_DRIVE;
                }

                break;

            case PATHFINDER_STATE_FINAL:
                finalSide.copyPath(final);
                if (runDirection == RUN_DIRECTION_CW) finalSide.mirrorPath();

                // Append the waypoints of the chosen path and correct for run direction
                for (const Waypoint& wp : finalSide.path.waypoints)
                {
                    guidanceData.appendWaypoint(wp);
                }

                pathfinderState = PATHFINDER_STATE_STOP;
                break;

            case PATHFINDER_STATE_STOP:
                if (guidanceData.getReachedLastWaypoint()) stop = true;
                break;
        }
    }

    size_t getRound() {return round;}

    bool shouldStop(){return stop;}

private:
    std::vector<Side> sides;
    Side initialSide;
    Side finalSide;
    size_t currentSideIndex;
    size_t round;
    bool stop;
    enum RUN_DIRECTION runDirection;
    enum PATHFINDER_STATE pathfinderState;

    Path getPathFromObstacle(const Obstacle& obs)
    {
        if(runDirection == RUN_DIRECTION_CCW)
        {
            if (obs.positionNumber <= 3)
            {
                if (obs.getColor() == OBSTACLE_COLOUR_RED) return lightOuter;
                if (obs.getColor() == OBSTACLE_COLOUR_GREEN) return fullInner;
            }
            if (obs.getColor() == OBSTACLE_COLOUR_RED) return fullOuter;
            if (obs.getColor() == OBSTACLE_COLOUR_GREEN) return lightInner;
        }
        else 
        {
            if (obs.positionNumber <= 3)
            {
                if (obs.getColor() == OBSTACLE_COLOUR_RED) return fullInner;
                if (obs.getColor() == OBSTACLE_COLOUR_GREEN) return lightOuter;
            }
            if (obs.getColor() == OBSTACLE_COLOUR_RED) return lightInner;
            if (obs.getColor() == OBSTACLE_COLOUR_GREEN) return fullOuter;
        
        }
    }

    bool inBox(Vec2f p, Vec2f lowerLeft, Vec2f upperRight)
    {
        if (p.x >= lowerLeft.x && p.x <= upperRight.x)
        {
            if (p.y >= lowerLeft.y && p.y <= upperRight.y) return true;
        }
        return false;
    }

    Path initial;
    Path final;
    Path fullInner;
    Path lightInner;
    Path lightOuter;
    Path fullOuter;

    void initPaths()
    {
        // Base Values
        const float xFirstWaypoint{0.9f};
        const float xSecondWaypoint{1.5f};
        const float xThirdWaypoint{2.0f};

        const float yFullInner{0.8f};
        const float yLightInner{0.7f};
        const float yLightOuter{0.3f};
        const float yFullOuter{0.2f};

        // Initial
        initial.name = "Initial";
        initial.waypoints.clear();
        initial.waypoints.push_back(Waypoint(Vec2f(2.0f, 0.5f), 0.0f, false));
        initial.waypoints.push_back(Waypoint(Vec2f(2.0f, 0.5f), 0.0f, false));
        initial.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.5f), 0.0f, false));
        initial.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.7f), 0.0f, true));
        initial.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.3f), 0.0f, true, true));

        // Final
        final.name = "Final";
        final.waypoints.clear();
        final.waypoints.push_back(Waypoint(Vec2f(1.0f, 0.5f), 0.0f, false));
        final.waypoints.push_back(Waypoint(Vec2f(1.5f, 0.5f), 0.0f, true));

        // Full inner
        fullInner.name = "Full inner";
        fullInner.waypoints.clear();
        fullInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullInner), M_PI/2.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullInner), 0.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullInner), 0.0f, true));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.3f), 0.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.6f, 0.7f), 0.0f, true));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.45f), 0.0f, true, true));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), 0.0f, true, true));

        // Light inner
        lightInner.name = "Light inner";
        lightInner.waypoints.clear();
        lightInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yLightInner), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightInner), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yLightInner), 0.0f, true));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.4f), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.6f, 0.6f), 0.0f, true));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.55f, 0.35f), 0.0f, true, true));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.58f, 0.2f), 0.0f, true, true));

        // Light outer
        lightOuter.name = "Light outer";
        lightOuter.waypoints.clear();
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yLightOuter), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightOuter), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yLightOuter), 0.0f, true));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(2.2f, 0.3f), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.6f), 0.0f, true));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.3f), 0.0f, true, true));

        // Full outer
        fullOuter.name = "Full outer";
        fullOuter.waypoints.clear();
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullOuter), 0.0f, true));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.2f), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.5f), 0.0f, true));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), 0.0f, true, true));
    }
};
    
