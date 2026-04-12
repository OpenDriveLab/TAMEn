#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <numeric>
#include "common.h"

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

int main()
{
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;
    
    ret = robot.login_in("10.5.5.100");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");


    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");
    printf("power on succeed\n");

    ret = robot.enable_robot();
    if (ret == ERR_SUCC)
    {
        printf("enable succeed!\n");
    }

    JointValue q[2] = {{1.57 ,0 ,0 ,0 ,0 ,0}, {1.57 ,0 ,0 ,0 ,0 ,0}};
    JointValue q2[2] = {{1.57 ,-1 ,0 ,0 ,0 ,0}, {1.57 ,-1 ,0 ,0 ,0 ,0}};

    MoveMode moveop[2] = {ABS, ABS};
    double vel[2] = {1, 1};
    double acc[2] = {1, 1};
    double tol[2] = {0, 0};
    double id[2] = {0, 0};
    ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, q, vel, acc);

    std::vector<double> time;
    int count = 0;
    for (int i = 0; i < 50; i++)
    {
        int error[2] = {0, 0};
        robot.robot_is_in_error(error);
        if (error[0] || error[1])
        {
            printf("Robot seems to have error!\n"); // Some servo errors cannot be cleared with clear_error
            robot.clear_error();
            robot.power_off();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));   
            robot.power_on();
        }
        else
        {
            printf("No error! got on test!\n");
        }
        printf("Try once\n");
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        ret = robot.enable_robot();
        if (ret == ERR_SUCC)
        {
            printf("enable succeed!\n");
            if (i % 2)
                ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, q2, vel, acc);
            else
                ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, q, vel, acc);
        }
        else
        {
            printf("ERROR = %d\n", ret);
            count++;
            i--;
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        std::cout << "Enable time: " << duration.count() << " seconds\n";
        time.push_back(duration.count());
        printf("Try Disable\n");
        ret = robot.disable_robot();
        // After disable, wait at least 1 second before enabling again to avoid hardware damage
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));   
    }

    double total2 = accumulate(time.begin(), time.end(), 0.0);
    printf("average consuming %lf\n", total2 / time.size());
    printf("failed times = %d\n", count);
    robot.login_out();
    return 0;
}
