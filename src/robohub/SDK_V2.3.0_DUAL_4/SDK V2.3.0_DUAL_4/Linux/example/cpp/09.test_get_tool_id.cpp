#include <JAKAZuRobot.h>
#include <chrono>
#include <iostream>
#include <thread>

#define JK_PI (3.141592653589793)

int main()
{
    JAKAZuRobot robot;
    errno_t ret = ERR_SUCC;
    int id = 0;

    ret = robot.login_in("192.168.2.200");
    std::cout << "login: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;

    while (true)
    {
        std::cout << "===========================\n";

        robot.get_tool_id(&id);
        std::cout << "id is " << id << std::endl;

        std::cout << "\n";
    }

    ret = robot.login_out();
    std::cout << "login_out: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << std::endl;

    return 0;
}