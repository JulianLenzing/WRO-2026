#pragma once

#include <iostream>

#include "State.h"

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
			std::cout << "Entered new State: " << current->name() << std::endl;
			current->enter(robot);
		}
    }

    void update(RobotSystem& robot)
    {
        if(current) current->update(robot);
    }
};
