#include <JAKAZuRobot.h>
#include <iostream>
#include "common.h"
#include "jktypes.h"

int main()
{
    JAKAZuRobot robot;
    int chn_mode;
    errno_t ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    ret = robot.get_rs485_chn_mode(0, &chn_mode);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "before get_rs485_chn_mode");
    std::cout << "get_rs485_chn_mode: id 0, mode " << chn_mode << std::endl;

    ret = robot.set_rs485_chn_mode(0, 0);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "set_rs485_chn_mode");
    std::cout << "set_rs485_chn_mode: id 0, mode 0" << std::endl;

    ret = robot.get_rs485_chn_mode(0, &chn_mode);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "after get_tio_pin_mode");
    std::cout << "get_rs485_chn_mode: id 0, mode " << chn_mode << std::endl;

    ret = robot.get_rs485_chn_mode(1, &chn_mode);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "after get_tio_pin_mode");
    std::cout << "get_rs485_chn_mode: id 1, mode " << chn_mode << std::endl;

    ModRtuComm rtu = {
        0,
        1,
        9600,
        8,
        1,
        78,
    };
    ret = robot.set_rs485_chn_comm(rtu);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "after set_rs485_chn_comm");

    ret = robot.login_out();
    return 0;
}