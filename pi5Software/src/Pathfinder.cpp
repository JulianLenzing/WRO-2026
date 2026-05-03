#include "Pathfinder.h"
#include "Run_Type.h"

Pathfinder::Pathfinder(const RUN_TYPE& pRunType) 
    : 
    runType(pRunType), 
    currentSideIndex(0), 
    runDirection(RUN_DIRECTION_CCW), 
    round(1), 
    pathfinderState(PATHFINDER_STATE_INITIAL),
    startedLeft(false),
    stop(false)
{
    sides.emplace_back(Vec2f(0.0f, 0.0f), float(0.0f), Vec2f(0.9f, 0.0f), Vec2f(2.1f, 1.0f), Vec2f(1.5f, 0.5f));
    sides.emplace_back(Vec2f(3.0f, 0.0f), float(M_PI/2.0f), Vec2f(2.0f, 0.9f), Vec2f(3.0f, 2.1f), Vec2f(2.5f, 1.5f));
    sides.emplace_back(Vec2f(3.0f, 3.0f), float(M_PI), Vec2f(0.9f, 2.0f), Vec2f(2.1f, 3.0f), Vec2f(1.5f, 2.5f));
    sides.emplace_back(Vec2f(0.0f, 3.0f), float(3.0f*M_PI/2.0f), Vec2f(0.0f, 0.9f), Vec2f(1.0f, 2.1f), Vec2f(0.5f, 1.5f));

    initialSide = sides[0];
    finalSide = sides[0];

    initPaths();
}

void Pathfinder::setStartingPosition(Vec2f position)
{
    if (position.x <= 1.5f)
    {
        startedLeft = true;

    }
    else startedLeft = false;
}

void Pathfinder::filterObstacles(std::vector<Obstacle> obstacles,  std::vector<Obstacle>& output){
    output.clear();
    for(int i = 0; i < 4; i++)
    {
        std::vector<Obstacle> sideObstacles;
        for (const Obstacle& obs : obstacles)
        {
            if (inBox(obs.position, sides[i].lowerLeft, sides[i].upperRight))
            {
                sideObstacles.push_back(obs);
            }
        }
        if (sideObstacles.size() <= 0) continue;
        Obstacle obstacle = sideObstacles[0];
        for (const Obstacle& obs : sideObstacles)
        {
            if (obs.count > obstacle.count) obstacle = obs;
        }
        output.push_back(obstacle);
    }
}

bool Pathfinder::getSideObstacle(std::vector<Obstacle> obstacles, int sideIndex, Obstacle& obstacle) {
    std::vector<Obstacle> sideObstacles;
    for (const Obstacle& obs : obstacles)
    {
        if (inBox(obs.position, sides[sideIndex].lowerLeft, sides[sideIndex].upperRight))
        {
            sideObstacles.push_back(obs);
        }
    }
    if (sideObstacles.size() <= 0) return false; // Wait until an obstacle in the current side was detected
    obstacle = sideObstacles[0];
    for (const Obstacle& obs : sideObstacles)
    {
        if (obs.count > obstacle.count) obstacle = obs;
    }
    return true;
}

void Pathfinder::update(Vec2f position, float heading, std::vector<Obstacle> obstacles, GuidanceData& guidanceData)
{
    if (runType == RUN_TYPE_OBSTACLE_RUN) // Configure for obstacle run
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
                Obstacle obstacle;
                if(!getSideObstacle(obstacles, currentSideIndex, obstacle)) return;
                if (obstacle.getColor() == OBSTACLE_COLOUR_UNKNOWN) return;

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
    else // Configure for opening run
    {
        switch (pathfinderState)
        {
        case PATHFINDER_STATE_INITIAL:
            initialSide.copyPath(openingRunInitial);
            if (runDirection == RUN_DIRECTION_CW) initialSide.mirrorPath();

            for (const Waypoint& wp : initialSide.path.waypoints)
            {
                guidanceData.appendWaypoint(wp);
            }

            // Index to next side
            if (runDirection == RUN_DIRECTION_CCW) currentSideIndex++;
            else currentSideIndex--;
            currentSideIndex = (currentSideIndex + 4) % 4;
            pathfinderState = PATHFINDER_STATE_SIDES;
            break;

        case PATHFINDER_STATE_SIDES:
            if (!sides[currentSideIndex].hasPath)
            {
                sides[currentSideIndex].copyPath(openingRunPath);
                if (runDirection == RUN_DIRECTION_CW) sides[currentSideIndex].mirrorPath();
            }

            for (const Waypoint& wp : sides[currentSideIndex].path.waypoints)
            {
                guidanceData.appendWaypoint(wp);
            }

            // Index to next side
            if (runDirection == RUN_DIRECTION_CCW) currentSideIndex++;
            else currentSideIndex--;
            currentSideIndex = (currentSideIndex + 4) % 4;
            if (currentSideIndex == 0) round++;

            if (round == ROUNDS_TO_DRIVE + 1)
            {
                pathfinderState = PATHFINDER_STATE_FINAL;
                round = ROUNDS_TO_DRIVE;
            }

            break;

        case PATHFINDER_STATE_FINAL:
            if (startedLeft)
            {
                if (runDirection == RUN_DIRECTION_CCW) finalSide.copyPath(openingRunFinalLeft);
                else finalSide.copyPath(openingRunFinalRight);
            }
            else
            {
                if (runDirection == RUN_DIRECTION_CCW) finalSide.copyPath(openingRunFinalRight);
                else finalSide.copyPath(openingRunFinalLeft);
            }
            
            if (runDirection == RUN_DIRECTION_CW) finalSide.mirrorPath();

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
}

Path Pathfinder::getPathFromObstacle(const Obstacle& obs)
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

void Pathfinder::initPaths()
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
        initial.waypoints.push_back(Waypoint(Vec2f(2.8f, 0.5f), 0.0f, true));
        initial.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), M_PI/2.0f, true, true));
        //turnWrapper(initial.waypoints);

        // Final
        final.name = "Final";
        final.waypoints.clear();
        final.waypoints.push_back(Waypoint(Vec2f(1.0f, 0.5f), 0.0f, false));
        final.waypoints.push_back(Waypoint(Vec2f(1.5f, 0.5f), 0.0f, true));

        // Full inner
        fullInner.name = "Full inner";
        fullInner.waypoints.clear();
        fullInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullInner), 0.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullInner), 0.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullInner), 0.0f, true));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.6f), toRad(-30), false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.8f, 0.5f), 0.0f, true));
        fullInner.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), M_PI/2.0f, false, true));
        //turnWrapper(fullInner.waypoints);

        // Light inner
        lightInner.name = "Light inner";
        lightInner.waypoints.clear();
        lightInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yLightInner), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightInner), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yLightInner), 0.0f, true));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.4f), toRad(-70), false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.6f, 0.6f), toRad(90), true));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.55f, 0.35f), toRad(90), true, true));
        lightInner.waypoints.push_back(Waypoint(Vec2f(2.58f, 0.2f), toRad(90), true, true));

        // Light outer
        lightOuter.name = "Light outer";
        lightOuter.waypoints.clear();
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yLightOuter), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightOuter), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yLightOuter), 0.0f, true));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(2.2f, 0.3f), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.6f), toRad(90), true));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.3f), toRad(90), true, true));

        // Full outer
        fullOuter.name = "Full outer";
        fullOuter.waypoints.clear();
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.2f), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.5f), toRad(90), true));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), toRad(90), true, true));


        // Opening run initial
        openingRunInitial.name = "Opening run initial";
        openingRunInitial.waypoints.clear();
        openingRunInitial.waypoints.push_back(Waypoint(Vec2f(2.0f, yLightOuter), 0.0f, false));
        openingRunInitial.waypoints.push_back(Waypoint(Vec2f(2.4f, yLightOuter), 0.0f, false));

        // Opening run final left
        openingRunFinalLeft.name = "Opening run final left";
        openingRunFinalLeft.waypoints.clear();
        openingRunFinalLeft.waypoints.push_back(Waypoint(Vec2f(1.0f, yLightOuter), 0.0f, false));
        openingRunFinalLeft.waypoints.push_back(Waypoint(Vec2f(1.25f, yLightOuter), 0.0f, true));

        // Opening run final right
        openingRunFinalRight.name = "Opening run final right";
        openingRunFinalRight.waypoints.clear();
        openingRunFinalRight.waypoints.push_back(Waypoint(Vec2f(1.0f, yLightOuter), 0.0f, false));
        openingRunFinalRight.waypoints.push_back(Waypoint(Vec2f(1.75f, yLightOuter), 0.0f, true));

        // Opening run path
        openingRunPath.name = "Opening run path";
        openingRunPath.waypoints.clear();
        openingRunPath.waypoints.push_back(Waypoint(Vec2f(0.6f, yLightOuter), 0.0f, false));
        openingRunPath.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightOuter), 0.0f, false));
        openingRunPath.waypoints.push_back(Waypoint(Vec2f(2.4f, yLightOuter), 0.0f, false));
    }
