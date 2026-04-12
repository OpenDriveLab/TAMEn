#include "jkerr.h"
#include "jktypes.h"
#include <JAKAZuRobot.h>
#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    JAKAZuRobot robot;
    errno_t ret = ERR_SUCC;
    robot.login_in("192.168.2.200");
    robot.power_on();
    robot.enable_robot();

    CartesianPose pos {436, 11.21, 33, 90, 0, 0};
    ret = robot.linear_move(&pos, MoveMode::ABS, true, 10);
    if (ret != ERR_SUCC)
    {
        std::cout << "linear move failed !.\n";
    }

    robot.login_out();
    return 0;
}