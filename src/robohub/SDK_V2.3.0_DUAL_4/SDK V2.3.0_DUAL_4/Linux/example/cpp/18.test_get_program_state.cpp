#include <JAKAZuRobot.h>
#include <iostream>

int main()
{
    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");

    ProgramState state;
    robot.get_program_state(&state);

    std::cout << "program state: " << state << "\n";

    robot.login_out();
    return 0;
}