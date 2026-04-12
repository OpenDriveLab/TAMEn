#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)

int main()
{
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;
    JointValue jstep_pos { { 0, 0, 0, 0, 5 / 100.0 * JK_PI, 0 } };
    JointValue jstep_neg { { 0, 0, 0, 0, -5 / 100.0 * JK_PI, 0 } };
    
    ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    while (true)
    {
        ret = robot.joint_move(&jstep_pos, MoveMode::INCR, false, 6.28);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos failed.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

        ret = robot.joint_move(&jstep_neg, MoveMode::INCR, false, 6.28);
        if (ret == ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg ok.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg failed.\n";
        }
    }

    robot.login_out();
    return 0;
}
