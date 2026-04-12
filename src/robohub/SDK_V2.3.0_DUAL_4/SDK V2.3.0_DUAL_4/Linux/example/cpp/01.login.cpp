#include <JAKAZuRobot.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>


int main()
{
    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");
    
    errno_t ret;
    int cnt = 1;
    while(1) 
    {
        std::cout << "loop cnt------- " << cnt << std::endl;
        ret = robot.power_on();
        if (eret == ERR_SUCC)
        {
            std::cout << "power_on success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "power_on failed: " << ret << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ret = robot.enable_robot();
        if (eret == ERR_SUCC)
        {
            std::cout << "enable_robot success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "enable_robot failed: " << ret << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        ret = servo_move_enable(true, 1, 0);
        ret = servo_move_enable(true, 1, 1);
        if (eret == ERR_SUCC)
        {
            std::cout << "servo_move_enable(true) success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "servo_move_enable(true) failed: " << ret << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ret = servo_move_enable(false,1, 0);
        ret = servo_move_enable(false,1, 1);
        if (eret == ERR_SUCC)
        {
            std::cout << "servo_move_enable(false) success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "servo_move_enable(false) failed: " << ret << std::endl;
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        ret = robot.drag_mode_enable(0, true);
        ret = robot.drag_mode_enable(1, true);
        if (eret == ERR_SUCC)
        {
            std::cout << "drag_mode_enable(true) success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "drag_mode_enable(true) failed: " << ret << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ret = drag_mode_enable(0, false);
        ret = drag_mode_enable(1, false);
        if (eret == ERR_SUCC)
        {
            std::cout << "drag_mode_enable(false) success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "drag_mode_enable(false) failed: " << ret << std::endl;
        }

        ret = robot.disable_robot();
        if (eret == ERR_SUCC)
        {
            std::cout << "power_on success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "power_on failed: " << ret << std::endl;
        }

        ret = robot.power_off();
        if (eret == ERR_SUCC)
        {
            std::cout << "enable_robot success: " << ret << std::endl;
        }
        else 
        {
             std::cout << "enable_robot failed: " << ret << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }


    robot.login_out();
    return 0;
}