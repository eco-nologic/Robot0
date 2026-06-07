#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include <Arduino.h>

class Robot;

class CommandDispatcher {
public:
    static CommandDispatcher& getInstance();

    void begin(Robot& robot);
    void dispatch(const String& cmd);

    CommandDispatcher(const CommandDispatcher&) = delete;
    CommandDispatcher& operator=(const CommandDispatcher&) = delete;

private:
    CommandDispatcher();
    Robot* _robot;
};

#endif // COMMAND_DISPATCHER_H
