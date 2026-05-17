#include "Pathfinder.h"
#include "Run_Type.h"

Pathfinder::Pathfinder(const float& pRobotLength, const RUN_TYPE& pRunType, const bool& pParkingObstacle)
    : 
    robotLength(pRobotLength),
    runType(pRunType),
    parkingObstacle(pParkingObstacle),
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

void Pathfinder::indexToNextSide()
{
    // Index to next side
    if (runDirection == RUN_DIRECTION_CCW) currentSideIndex++;
    else currentSideIndex--;
    currentSideIndex = (currentSideIndex + 4) % 4;
    if (currentSideIndex == 0) round++;
}

int Pathfinder::getNextSideIndex()
{    
    int index = currentSideIndex;
    if (runDirection == RUN_DIRECTION_CCW) index++;
    else index--;
    index = (index + 4) % 4;
    return index;
}

void Pathfinder::appendPath(GuidanceData& guidanceData, const Path& path)
{
    printf("Appended path: %s\n", path.name.c_str());
    for (const Waypoint& wp : path.waypoints)
    {
        guidanceData.appendWaypoint(wp);
    }
}

void Pathfinder::update(Vec2f position, float heading, std::vector<Obstacle> obstacles, GuidanceData& guidanceData)
{
    if (runType == RUN_TYPE_OBSTACLE_RUN) // Configure for obstacle run
    {
        switch (pathfinderState)
        {
        case PATHFINDER_STATE_INITIAL:
        {
            Obstacle obstacle;
            getSideObstacle(obstacles, 0, obstacle); // May proceed without valid obstacle to enable special cases; Non valid obstacles will be handeled in the getInitialPathFromObstacle function

            Path currentPath;
            if(!getInitialPathFromObstacle(currentPath, obstacle)) return;
            currentPath = initialSide.orientPath(currentPath);
            if (runDirection == RUN_DIRECTION_CW) initialSide.mirrorPath(currentPath);            
            appendPath(guidanceData, currentPath);
            
            Path cornerPath = getCornerPath(sides[getNextSideIndex()].obstacle);        
            cornerPath = initialSide.orientPath(cornerPath);
            if (runDirection == RUN_DIRECTION_CW) initialSide.mirrorPath(cornerPath);
            appendPath(guidanceData, cornerPath);
            
            indexToNextSide();
            pathfinderState = PATHFINDER_STATE_SIDES;
        }
            break;

        case PATHFINDER_STATE_SIDES:
        {            
            if(!sides[currentSideIndex].obstacle.isValid())
            {
                Obstacle obstacle;
                if(!getSideObstacle(obstacles, currentSideIndex, obstacle)) return;
                if (obstacle.getColor() == OBSTACLE_COLOUR_UNKNOWN) return;
                sides[currentSideIndex].obstacle = obstacle;
            }

            Path currentPath;
            currentPath = sides[currentSideIndex].orientPath(getPathFromObstacle(sides[currentSideIndex].obstacle, currentSideIndex));
            if (runDirection == RUN_DIRECTION_CW) sides[currentSideIndex].mirrorPath(currentPath);            
            appendPath(guidanceData, currentPath);
            
            Path cornerPath = getCornerPath(sides[getNextSideIndex()].obstacle);        
            cornerPath = sides[currentSideIndex].orientPath(cornerPath);
            if (runDirection == RUN_DIRECTION_CW) sides[currentSideIndex].mirrorPath(cornerPath);
            appendPath(guidanceData, cornerPath);            

            indexToNextSide();
            
            if (round == ROUNDS_TO_DRIVE + 1)
            {
                pathfinderState = PATHFINDER_STATE_FINAL;
                round = ROUNDS_TO_DRIVE;
            }
        }
            break;

        case PATHFINDER_STATE_FINAL:
        {
            Obstacle obstacle;
            if(!getSideObstacle(obstacles, 0, obstacle)) return;
            if (obstacle.getColor() == OBSTACLE_COLOUR_UNKNOWN) return;

            Path currentPath;
            currentPath = finalSide.orientPath(getFinalPathFromObstacle(obstacle));
            if (runDirection == RUN_DIRECTION_CW) finalSide.mirrorPath(currentPath);
            appendPath(guidanceData, currentPath);

            pathfinderState = PATHFINDER_STATE_STOP;
        }
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
        {
            Path currentPath = initialSide.orientPath(openingRunInitial);
            if (runDirection == RUN_DIRECTION_CW) initialSide.mirrorPath(currentPath);

            appendPath(guidanceData, currentPath);

            indexToNextSide();
            pathfinderState = PATHFINDER_STATE_SIDES;
        }
            break;

        case PATHFINDER_STATE_SIDES:            
        {
            Path currentPath = sides[currentSideIndex].orientPath(openingRunPath);
            if (runDirection == RUN_DIRECTION_CW) sides[currentSideIndex].mirrorPath(currentPath);
            
            appendPath(guidanceData, currentPath);

            indexToNextSide();

            if (round == ROUNDS_TO_DRIVE + 1)
            {
                pathfinderState = PATHFINDER_STATE_FINAL;
                round = ROUNDS_TO_DRIVE;
            }
        }
            break;

        case PATHFINDER_STATE_FINAL:
        {
            Path currentPath;
            if (startedLeft)
            {
                if (runDirection == RUN_DIRECTION_CCW) currentPath = finalSide.orientPath(openingRunFinalLeft);
                else currentPath = finalSide.orientPath(openingRunFinalRight);
            }
            else
            {
                if (runDirection == RUN_DIRECTION_CCW) currentPath = finalSide.orientPath(openingRunFinalRight);
                else currentPath = finalSide.orientPath(openingRunFinalLeft);
            }
            
            if (runDirection == RUN_DIRECTION_CW) finalSide.mirrorPath(currentPath);

            appendPath(guidanceData, currentPath);

            pathfinderState = PATHFINDER_STATE_STOP;
         }
            break;

        case PATHFINDER_STATE_STOP:
            if (guidanceData.getReachedLastWaypoint()) stop = true;
            break;
        }
    }
}

bool Pathfinder::getInitialPathFromObstacle(Path& path, const Obstacle& obs)
{
    if((startedLeft && runDirection == RUN_DIRECTION_CW) || (!startedLeft && runDirection == RUN_DIRECTION_CCW)) {path = initial; return true;}

    if (!obs.isValid()) return false;

    if(parkingObstacle)
    {
        if(runDirection == RUN_DIRECTION_CCW) 
        {
            if (obs.positionNumber == 3 || obs.positionNumber == 6) {
                if (obs.getColor() == OBSTACLE_COLOUR_RED) {path = parkingInitialOuter; return true;}
                else if(obs.getColor() == OBSTACLE_COLOUR_GREEN) {path = initialInner; return true;}
            }
            else {path = initial; return true;}
        }
        else
        {
            if (obs.positionNumber == 1 || obs.positionNumber == 4) {
                if (obs.getColor() == OBSTACLE_COLOUR_RED) {path = initialInner; return true;}
                else if(obs.getColor() == OBSTACLE_COLOUR_GREEN) {path = parkingInitialOuter; return true;}
            }
            else {path = initial; return true;}
        }
        return false; 
    }
    else
    {
        if(runDirection == RUN_DIRECTION_CCW) 
        {
            if (obs.positionNumber == 3 || obs.positionNumber == 6) {
                if (obs.getColor() == OBSTACLE_COLOUR_RED) {path = initialOuter; return true;}
                else if(obs.getColor() == OBSTACLE_COLOUR_GREEN) {path = initialInner; return true;}
            }
            else {path = initial; return true;}
        }
        else
        {
            if (obs.positionNumber == 1 || obs.positionNumber == 4) {
                if (obs.getColor() == OBSTACLE_COLOUR_RED) {path = initialInner; return true;}
                else if(obs.getColor() == OBSTACLE_COLOUR_GREEN) {path = initialOuter; return true;}
            }
            else {path = initial; return true;}
        }
    }
    return false; 
}

Path Pathfinder::getFinalPathFromObstacle(const Obstacle& obs)
{
    if (parkingObstacle)
    {
        if (runDirection == RUN_DIRECTION_CCW) return parkingFinalCCW;
        else return parkingFinalCW;
    }

    if (runDirection == RUN_DIRECTION_CCW)
    {
        if (obs.getColor() == OBSTACLE_COLOUR_RED)
        {
            if (startedLeft) return finalOuterLeft;
            else return finalOuterRight;
        }
        else
        {
            if (startedLeft) return finalInnerLeft;
            else return finalInnerRight;
        }
    }
    else
    {
        if (obs.getColor() == OBSTACLE_COLOUR_RED)
        {
            if (startedLeft) return finalInnerRight;
            else return finalInnerLeft;
        }
        else
        {
            if (startedLeft) return finalOuterRight;
            else return finalOuterLeft;
        }
    }
    return finalOuterLeft; // Suppress compiler warning
}

Path Pathfinder::getPathFromObstacle(const Obstacle& obs, int sideIndex)
{
    if (parkingObstacle && sideIndex == 0)
    {
        if (runDirection == RUN_DIRECTION_CCW)
        {
            if (obs.getColor() == OBSTACLE_COLOUR_RED) return parkingOuter;
            else return parkingInner;
        }
        else
        {
            if (obs.getColor() == OBSTACLE_COLOUR_RED) return parkingInner;
            else return parkingOuter;
        }
    }

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
    return fullOuter; // Suppress compiler warning
}

Path Pathfinder::getCornerPath(const Obstacle& nextObs)
{
    if(!nextObs.isValid()) return nextImmediate;
    
    if(runDirection == RUN_DIRECTION_CCW)
    {
        if(nextObs.positionNumber == 1 || nextObs.positionNumber == 4) return nextImmediate;
        else return nextLate;
    }
    else
    {
        if(nextObs.positionNumber == 3 || nextObs.positionNumber == 6) return nextImmediate;
        else return nextLate;
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
        
        // Corner paths
        /*
        nextImmediateCurrentFullInner.name = "Next immediate current full inner";
        nextImmediateCurrentFullInner.waypoints.clear();
        nextImmediateCurrentFullInner.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.6f), toRad(-30), false));
        nextImmediateCurrentFullInner.waypoints.push_back(Waypoint(Vec2f(2.8f, 0.5f), 0.0f, true));
        nextImmediateCurrentFullInner.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), M_PI/2.0f, false, true));
        
        nextImmediateCurrentLightInner.name = "Next immediate current light inner";
        nextImmediateCurrentLightInner.waypoints.clear();        
        nextImmediateCurrentLightInner.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.4f), toRad(-70), false));
        nextImmediateCurrentLightInner.waypoints.push_back(Waypoint(Vec2f(2.6f, 0.6f), toRad(90), true));
        nextImmediateCurrentLightInner.waypoints.push_back(Waypoint(Vec2f(2.55f, 0.35f), toRad(90), true, true));
        nextImmediateCurrentLightInner.waypoints.push_back(Waypoint(Vec2f(2.58f, 0.2f), toRad(90), true, true));
        
        nextImmediateCurrentFullOuter.name = "Next immediate current full outer";
        nextImmediateCurrentFullOuter.waypoints.clear();        
        nextImmediateCurrentFullOuter.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.2f), 0.0f, false));
        nextImmediateCurrentFullOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.5f), toRad(90), true));
        nextImmediateCurrentFullOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), toRad(90), true, true));
        
        nextImmediateCurrentLightOuter.name = "Next immediate current light outer";
        nextImmediateCurrentLightOuter.waypoints.clear();     
        nextImmediateCurrentLightOuter.waypoints.push_back(Waypoint(Vec2f(2.2f, 0.3f), 0.0f, false));
        nextImmediateCurrentLightOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.6f), toRad(90), true));
        nextImmediateCurrentLightOuter.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.3f), toRad(90), true, true));
        */
        
        nextImmediate.name = "Next immediate";
        nextImmediate.waypoints.clear();
        nextImmediate.waypoints.push_back(Waypoint(Vec2f(2.3f, 0.6f), toRad(-30), false));
        nextImmediate.waypoints.push_back(Waypoint(Vec2f(2.8f, 0.5f), 0.0f, true));
        nextImmediate.waypoints.push_back(Waypoint(Vec2f(2.5f, 0.2f), M_PI/2.0f, false, true));        
        
        nextLate.name = "Next late";
        nextLate.waypoints.clear();
        nextLate.waypoints.push_back(Waypoint(Vec2f(2.5f, 1.1f), M_PI/2.0f, false));

        // Unparking path CCW
        unparkingCCW.name = "Unparking path CCW";
        unparkingCCW.waypoints.clear();
        unparkingCCW.waypoints.push_back(Waypoint(Vec2f(2.0f - robotLength / 2.0f - 0.1f, 0.5f), M_PI/2.0f, true, false, 0.3f, 0.0f)); // Speed of 0.0f will become MIN_THROTTLE
        unparkingCCW.waypoints.push_back(Waypoint(Vec2f(2.0f - robotLength / 2.0f - 0.1f, 0.3f), M_PI/2.0f, true, true));
        unparkingCCW.waypoints.push_back(Waypoint(Vec2f(2.0f, 0.5f), 0.0f, false));

        // Unparking path CW
        unparkingCW.name = "Unparking path CW";
        unparkingCW.waypoints.clear();
        unparkingCW.waypoints.push_back(Waypoint(Vec2f(2.0f - robotLength / 2.0f - 0.1f, 0.5f), M_PI/2.0f, true, false, 0.3f, 0.0f)); // Speed of 0.0f will become MIN_THROTTLE
        unparkingCW.waypoints.push_back(Waypoint(Vec2f(2.0f - robotLength / 2.0f - 0.1f, 0.3f), M_PI/2.0f, true, true));
        unparkingCW.waypoints.push_back(Waypoint(Vec2f(1.5f, 0.5f), M_PI, false));

        // Initial
        initial.name = "Initial";
        initial.waypoints.clear();
        initial.waypoints.push_back(Waypoint(Vec2f(2.0f, 0.5f), 0.0f, false));
        
        // Initial outer
        initialOuter.name = "Initial outer";
        initialOuter.waypoints.clear();
        initialOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullOuter), 0.0f, false));

        // Initial inner
        initialInner.name = "Initial inner";
        initialInner.waypoints.clear();
        initialInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullInner), 0.0f, true));

        // Final outer left
        finalOuterLeft.name = "Final outer left";
        finalOuterLeft.waypoints.clear();
        finalOuterLeft.waypoints.push_back(Waypoint(Vec2f(1.0f, yFullOuter), 0.0f, false));
        finalOuterLeft.waypoints.push_back(Waypoint(Vec2f(1.25f, yFullOuter), 0.0f, true));

        // Final outer right
        finalOuterRight.name = "Final outer right";
        finalOuterRight.waypoints.clear();
        finalOuterRight.waypoints.push_back(Waypoint(Vec2f(1.0f, yFullOuter), 0.0f, false));
        finalOuterRight.waypoints.push_back(Waypoint(Vec2f(1.75f, yFullOuter), 0.0f, true));

        // Final inner left
        finalInnerLeft.name = "Final inner left";
        finalInnerLeft.waypoints.clear();
        finalInnerLeft.waypoints.push_back(Waypoint(Vec2f(1.0f, yFullInner), 0.0f, false));
        finalInnerLeft.waypoints.push_back(Waypoint(Vec2f(1.25f, yFullInner), 0.0f, true));

        // Final inner right
        finalInnerRight.name = "Final inner right";
        finalInnerRight.waypoints.clear();
        finalInnerRight.waypoints.push_back(Waypoint(Vec2f(1.0f, yFullInner), 0.0f, false));
        finalInnerRight.waypoints.push_back(Waypoint(Vec2f(1.75f, yFullInner), 0.0f, true));

        // Full inner
        fullInner.name = "Full inner";
        fullInner.waypoints.clear();
        fullInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullInner), 0.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullInner), 0.0f, false));
        fullInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullInner), 0.0f, false));        

        // Light inner
        lightInner.name = "Light inner";
        lightInner.waypoints.clear();
        lightInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yLightInner), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightInner), 0.0f, false));
        lightInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yLightInner), 0.0f, false));        

        // Light outer
        lightOuter.name = "Light outer";
        lightOuter.waypoints.clear();
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yLightOuter), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yLightOuter), 0.0f, false));
        lightOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yLightOuter), 0.0f, false));        

        // Full outer
        fullOuter.name = "Full outer";
        fullOuter.waypoints.clear();
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullOuter), 0.0f, false));
        fullOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullOuter), 0.0f, false));        

        // Parking initial outer
        parkingInitialOuter.name = "Parking initial outer";
        parkingInitialOuter.waypoints.clear();
        parkingInitialOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, 0.4f), 0.0f, false));

        // Parking inner
        parkingInner.name = "Parking inner";
        parkingInner.waypoints.clear();
        parkingInner.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullInner), 0.0f, false));
        parkingInner.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, yFullInner), 0.0f, false));
        parkingInner.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, yFullInner), 0.0f, false));

        // Parking outer
        parkingOuter.name = "Parking outer";
        parkingOuter.waypoints.clear();
        parkingOuter.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, 0.4f), 0.0f, false));
        parkingOuter.waypoints.push_back(Waypoint(Vec2f(xSecondWaypoint, 0.4f), 0.0f, false));
        parkingOuter.waypoints.push_back(Waypoint(Vec2f(xThirdWaypoint, 0.4f), 0.0f, false));

        float halfParkingZoneLength = robotLength * 1.5f / 2.0f;
        // Parking final ccw
        parkingFinalCCW.name = "Parking final CCW";
        parkingFinalCCW.waypoints.clear();
        parkingFinalCCW.waypoints.push_back(Waypoint(Vec2f(xFirstWaypoint, yFullInner), 0.0f, false));
        parkingFinalCCW.waypoints.push_back(Waypoint(Vec2f(1.5f, yFullInner), 0.0f, true));
        parkingFinalCCW.waypoints.push_back(Waypoint(Vec2f(2.0f - halfParkingZoneLength, 0.1f), toRad(-90), true, false, 0.5f, 0.1f)); // Max throttle of 0.1f will be replaced with min throttle in guidance

        // Parking final cw
        // To not change the existing waypoint append scheme this must be implemented as if the parking zone was on the left
        parkingFinalCW.name = "Parking final CW";
        parkingFinalCW.waypoints.clear();
        parkingFinalCW.waypoints.push_back(Waypoint(Vec2f(1.04f, yFullInner), 0.0f, false));
        parkingFinalCW.waypoints.push_back(Waypoint(Vec2f(1.0f + halfParkingZoneLength, 0.1f), toRad(-90), true, false, 0.5f, 0.1f)); // Max throttle of 0.1f will be replaced with min throttle in guidance
        
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
