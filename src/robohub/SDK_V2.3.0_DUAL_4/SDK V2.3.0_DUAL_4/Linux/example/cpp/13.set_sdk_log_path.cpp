#include <JAKAZuRobot.h>
#include <iostream>
#include "common.h"

int main()
{
    errno_t ret = JAKAZuRobot::static_Set_SDK_filepath(".\\log");
    ASSERT_TRUE_OR_EXIT(ret == 0, "static_Set_SDK_filepath");

    JAKAZuRobot robot;
    ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_EXIT(ret == 0, "login");

    ret = robot.set_SDK_filepath(".\\test.log");

    ret = robot.login_out();
    ASSERT_TRUE_OR_EXIT(ret == 0, "login");

    return 0;
}