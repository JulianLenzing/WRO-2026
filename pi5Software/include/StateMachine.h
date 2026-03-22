#pragma once

#include <iostream>
#include <chrono>

#include "State.h"
#include "RobotSystem.h"

class StateMachine
{
private:
    State* current = nullptr;

public:
    void setState(State* newState, RobotSystem& robot)
    {
        if(current) current->exit(robot);

        current = newState;

        if(current) 
        {
            std::chrono::milliseconds duration;
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime);
			std::cout << duration.count() << " ms - " << "Entered new State: " << current->name() << std::endl;
			current->enter(robot);
		}
    }

    void update(RobotSystem& robot)
    {
        if(current) current->update(robot);
    }
};
