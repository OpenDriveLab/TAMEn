#include <JAKAZuRobot.h>
#include <iostream>
#include "common.h"

int main()
{
    JAKAZuRobot robot;
    int pin_mode;
    errno_t ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    ret = robot.get_tio_pin_mode(1, &pin_mode);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "before get_tio_pin_mode");
    std::cout << "get_tio_pin_mode: " << pin_mode << std::endl;

    ret = robot.set_tio_pin_mode(1, 0xFF);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "get_tio_pin_mode");
    std::cout << "set_tio_pin_mode: [type 1, mode 0xFF]" << std::endl;

    ret = robot.get_tio_pin_mode(1, &pin_mode);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "after get_tio_pin_mode");
    std::cout << "get_tio_pin_mode: " << pin_mode << std::endl;

    ret = robot.login_out();
    return 0;
}