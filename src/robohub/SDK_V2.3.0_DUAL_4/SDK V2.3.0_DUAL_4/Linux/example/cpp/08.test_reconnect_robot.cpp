#include <JAKAZuRobot.h>
#include <chrono>
#include <iostream>
#include <thread>

#define JK_PI (3.141592653589793)

int main()
{
    JAKAZuRobot robot;
    errno_t ret = ERR_SUCC;

    while (true)
    {
        std::cout << "===========================\n";

        ret = robot.login_in("192.168.2.200");
        std::cout << "login: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;

        ret = robot.power_on();
        std::cout << "power_on: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;

        ret = robot.enable_robot();
        std::cout << "enable_robot: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        ret = robot.disable_robot();
        std::cout << "disable_robot: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;

        ret = robot.power_off();
        std::cout << "disable_robot: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        ret = robot.login_out();
        std::cout << "login_out: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "\n";
    }

    return 0;
}