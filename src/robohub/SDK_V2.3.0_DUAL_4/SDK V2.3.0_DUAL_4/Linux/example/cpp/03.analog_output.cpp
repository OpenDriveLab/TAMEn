#include <JAKAZuRobot.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <ratio>
#include <thread>
#include <cassert>
#include <vector>

errno_t test1(JAKAZuRobot& robot)
{
    errno_t ret = ERR_SUCC;
    float cab_ao_1;
    size_t times = 100;
    while (times--)
    {
        float target = times;
        std::cout << " >>>>>>>> set target " << target << " >>>>>>>>>>\n";
        ret = robot.set_analog_output(IOType::IO_CABINET, 6, target);
        assert(ret == ERR_SUCC);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        robot.get_analog_output(IOType::IO_CABINET, 6, &cab_ao_1);

        std::cout << cab_ao_1 << " -- " << target << "\n";
        assert(std::fabs(cab_ao_1 - target) < 0.1);
    }

    return ret;
}


errno_t test2(JAKAZuRobot& robot)
{
    errno_t ret = ERR_SUCC;
    float cab_ao_1;
    size_t times = 100;

    while (times--)
    {
        float target = times;
        // std::cout << " >>>>>>>> set target " << target << " >>>>>>>>>>\n";
        ret = robot.set_analog_output(IOType::IO_CABINET, 6, target);
        assert(ret == ERR_SUCC);

        auto st = std::chrono::system_clock::now();
        while (true)
        {
            ret = robot.get_analog_output(IOType::IO_CABINET, 6, &cab_ao_1);
            assert(ret == ERR_SUCC);
            if (std::fabs(cab_ao_1 - target) < 0.1) break;
            // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        };

        auto ed = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> dur = ed - st;

        std::cout << "duration-" << times << ": " << dur.count() << "ms" << std::endl;
    }

    return ret;
}

int main(int argc, char**)
{
    JAKAZuRobot robot;
    errno_t ret =     robot.login_in("192.168.2.200");
    assert(ret == ERR_SUCC);

    if (argc > 1)
    {
        test1(robot);
    }
    else
    {
        test2(robot);
    }
    
    ret = robot.login_out();
    assert(ret == ERR_SUCC);

    return 0;
}
