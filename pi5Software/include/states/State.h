#pragma once

#include <string>

class RobotSystem;

class State
{
public:
    virtual void enter(RobotSystem& robot) {}
    virtual void update(RobotSystem& robot) = 0;
    virtual void exit(RobotSystem& robot) {}
    virtual std::string name() const = 0;
    virtual ~State() {}
};
