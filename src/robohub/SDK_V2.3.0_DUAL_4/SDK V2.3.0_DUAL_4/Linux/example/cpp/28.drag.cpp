#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

int main()
{
    JAKAZuRobot robot;
    errno_t ret = robot.login_in("10.5.5.100");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    robot.power_off();

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    robot.clear_error();

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    ret = robot.drag_mode_enable(LEFT, 1);
    if (ret != ERR_SUCC)
    {
        std::cout << "left robot drag mode enable failed" << std::endl;
    }

    BOOL is_drag[2];
    for(int i = 0; i < 20; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ret = robot.is_in_drag_mode(is_drag);
        if (ret != ERR_SUCC)
        {
            std::cout << "get drag mode failed" << std::endl;
            break;
        }
        std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl;
    }
    ret = robot.drag_mode_enable(LEFT, 0);
    if (ret != ERR_SUCC)
    {
        std::cout << "left robot drag mode disable failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.is_in_drag_mode(is_drag);
    std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl;

    robot.login_out();
    return 0;
}
