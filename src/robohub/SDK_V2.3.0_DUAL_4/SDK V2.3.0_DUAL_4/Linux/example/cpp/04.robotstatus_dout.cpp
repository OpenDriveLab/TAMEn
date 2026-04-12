#include <JAKAZuRobot.h>
#include <cassert>
#include <iostream>

int main()
{
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;
    
    ret = robot.login_in("192.168.2.200");
    assert(ret == ERR_SUCC);
    
    ret = robot.get_robot_status(&robotStatus);
    assert(ret == ERR_SUCC);

    std::cout << "digital output: " << std::endl;
    for (int i = 0; i < 256; i++)
    {
        std::cout << robotStatus.dout[i] << ",";
    }

    std::cout << std::endl << "digital input: " << std::endl;
    for (int i = 0; i < 256; i++)
    {
        std::cout << robotStatus.din[i] << ",";
    }

    std::cout << std::endl << "extio: " << std::endl;
    for (int i = 0; i < 256; i++)
    {
        std::cout << robotStatus.extio.din[i] << ",";
    }

    std::cout << std::endl;

    robot.login_out();
    return 0;
}