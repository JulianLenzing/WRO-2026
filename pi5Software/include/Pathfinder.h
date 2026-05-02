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
#include "Run_Type.h"
#include "Line.h"

#define ROUNDS_TO_DRIVE 3
#define WAYPOINT_INTERPOLATION_COUNT 15
#define MIN_TURN_RADIUS 0.2f

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

            printf("Old heading: %.2f\n", wp.heading);
            // Reflect heading across perpendicular to d (matches point mirroring)
            Vec2f hv(cosf(wp.heading), sinf(wp.heading));

            float dot = hv.x * d.x + hv.y * d.y;
            Vec2f hv_mirrored = hv - d * (2.0f * dot);  // reflect across perp of d

            wp.heading = atan2f(hv_mirrored.y, hv_mirrored.x);
            if (wp.heading < 0) wp.heading += 2.0f * M_PI;
            printf("New Heading: %.2f\n", wp.heading);
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
    Pathfinder(const RUN_TYPE& pRunType);

    void update(Vec2f position, float heading, std::vector<Obstacle> obstacles, GuidanceData& guidanceData);
    
    void filterObstacles(std::vector<Obstacle> obstacles,  std::vector<Obstacle>& output);
    
    bool getSideObstacle(std::vector<Obstacle> obstacles, int sideIndex, Obstacle& obstacle);

    void setRunDirection(enum RUN_DIRECTION pRunDirection) {runDirection = pRunDirection;}

    void setStartingPosition(Vec2f position);

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
    const RUN_TYPE runType;
    bool startedLeft;

    Path getPathFromObstacle(const Obstacle& obs);

    bool inBox(Vec2f p, Vec2f lowerLeft, Vec2f upperRight)
    {
        if (p.x >= lowerLeft.x && p.x <= upperRight.x)
        {
            if (p.y >= lowerLeft.y && p.y <= upperRight.y) return true;
        }
        return false;
    }

    // Obstacle run
    Path initial;
    Path final;
    Path fullInner;
    Path lightInner;
    Path lightOuter;
    Path fullOuter;

    // Opening run
    Path openingRunInitial;
    Path openingRunFinalLeft;
    Path openingRunFinalRight;
    Path openingRunPath;

    void initPaths();

    float normaliseAngle(float angle)
    {
        return fmodf(fmodf(angle, 2.0f*M_PI)+2.0f*M_PI, 2.0f*M_PI);
    }

    void turnWrapper(std::vector<Waypoint>& output)
    {
        if (output.size() >= 2)
        {
            Waypoint wp2 = output.back();
            output.pop_back();
            generateCircleSection(output.back(), wp2, output);
        }
    }

    float toRad(float degrees)
    {
        return degrees * M_PI / 180.0f;
    }

    void generateCircleSection(Waypoint wp1, Waypoint wp2, std::vector<Waypoint>& output)
    {
        if (wp2.reverse)
        {
            wp1.heading += M_PI;
            wp2.heading += M_PI;
        }

        wp1.heading = normaliseAngle(wp1.heading);
        wp2.heading = normaliseAngle(wp2.heading);

        Line n1(wp1.point, wp1.point + Vec2f(cosf(wp1.heading+M_PI/2.0f), sinf(wp1.heading+M_PI/2.0f)));
        Line n2(wp2.point, wp2.point + Vec2f(cosf(wp2.heading+M_PI/2.0f), sinf(wp2.heading+M_PI/2.0f)));

        Line l1(wp1.point, wp1.point + Vec2f(cosf(wp1.heading), sinf(wp1.heading)));
        Line l2(wp2.point, wp2.point + Vec2f(cosf(wp2.heading), sinf(wp2.heading)));

        std::optional<Vec2f> center;
        if (normaliseAngle(wp1.heading+M_PI) > normaliseAngle(wp2.heading+0.05) || normaliseAngle(wp1.heading+M_PI) < normaliseAngle(wp2.heading-0.05))
        {
            center = Line::intersectionInfinite(n1, n2);
        }
        else
        {
            center = (wp1.point+wp2.point) / 2.0f;
        }
        if (!center.has_value()) return;

        Vec2f rel1 = wp1.point - center.value();
        Vec2f rel2 = wp2.point - center.value();
        float radius1 = rel1.length();
        float radius2 = rel2.length();

        if (radius1 < radius2 - 0.01f || radius2 > radius1 + 0.01f)
        {
            printf("Radius missmatch!\n");
            return;
        }

        float radius = radius1;
        if (radius < MIN_TURN_RADIUS)
        {
            printf("Warning! Radius smaller than minimum turn radius!\n");
        }

        float circumference = 2.0f * radius * M_PI;
        float sectionSize;
        bool ccw = true;
        if ((rel1.x > 0 && wp1.heading > M_PI) || (rel1.x < 0 && wp1.heading < M_PI) || (rel1.y > 0 && wp1.heading == 0.0f) || (rel1.y < 0 && wp1.heading == M_PI))
        {
            ccw = false;
        }

        if (ccw) sectionSize = normaliseAngle(atan2f(rel1.x, rel1.y) - atan2f(rel2.x, rel2.y)) / (2.0f*M_PI);
        else sectionSize = normaliseAngle(atan2f(rel2.x, rel2.y) - atan2f(rel1.x, rel1.y)) / (2.0f*M_PI);

        Waypoint wp(wp2);
        wp.point = wp1.point;
        wp.heading = wp1.heading;

        int interpolationCount = std::clamp(int(sectionSize * 50.0f), 5, 99999);
        float distance = circumference * sectionSize / float(interpolationCount);
        float angle = 2.0f*M_PI * sectionSize / float(interpolationCount);
        if (!ccw) angle = -angle;
        for (int i = 0; i < interpolationCount; i++)
        {
            float middleHeading = normaliseAngle(wp.heading + angle/2.0f);
            wp.heading += angle;
            wp.heading = normaliseAngle(wp.heading);

            wp.point += Vec2f(cosf(middleHeading), sinf(middleHeading)) * distance;

            Waypoint copy = wp;
            if (wp2.reverse) copy.heading = fmodf(copy.heading + M_PI, 2.0f*M_PI); // Return with heading facing the right way again
            output.push_back(copy);
        }

    }
};
    
