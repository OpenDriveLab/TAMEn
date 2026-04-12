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

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    robot.robot_zero_ftsensor(1);
    robot.robot_zero_ftsensor(2);

    double deadzone[6] = {.1,.2,.3,.4,.5,.6};
    ret = robot.robot_set_ftsensor_deadzone(1, deadzone);
    if (ret != ERR_SUCC)
    {
        std::cout << "set deadzone failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ret = robot.robot_get_ftsensor_deadzone(1, deadzone);
    if (ret != ERR_SUCC)
    {
        std::cout << "get deadzone failed" << std::endl;
    }
    else
    {
        printf("deadzone: %f %f %f %f %f %f\n", deadzone[0], deadzone[1], deadzone[2], deadzone[3], deadzone[4], deadzone[5]);
    }
    
    double deadzone_right[6] = {.2, 0.3, 0.4, 0.5, 0.6, 0.7};

    ret = robot.robot_set_ftsensor_deadzone(2, deadzone_right);
    if (ret != ERR_SUCC)
    {
        std::cout << "set deadzone failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ret = robot.robot_get_ftsensor_deadzone(2, deadzone_right);
    if (ret != ERR_SUCC)
    {
        std::cout << "get deadzone failed" << std::endl;
    }
    else
    {
        printf("deadzone: %f %f %f %f %f %f\n", deadzone_right[0], deadzone_right[1], deadzone_right[2], deadzone_right[3], deadzone_right[4], deadzone_right[5]);
    }

    ret = robot.robot_set_ftsensor_deadzone(1, deadzone_right);
    if (ret != ERR_SUCC)
    {
        std::cout << "set deadzone failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ret = robot.robot_get_ftsensor_deadzone(1, deadzone_right);
    if (ret != ERR_SUCC)
    {
        std::cout << "get deadzone failed" << std::endl;
    }
    else
    {
        printf("deadzone: %f %f %f %f %f %f\n", deadzone_right[0], deadzone_right[1], deadzone_right[2], deadzone_right[3], deadzone_right[4], deadzone_right[5]);
    }
    
    ret = robot.robot_set_ftsensor_deadzone(2, deadzone);
    if (ret != ERR_SUCC)
    {
        std::cout << "set deadzone failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ret = robot.robot_get_ftsensor_deadzone(2, deadzone);
    if (ret != ERR_SUCC)
    {
        std::cout << "get deadzone failed" << std::endl;
    }
    else
    {
        printf("deadzone: %f %f %f %f %f %f\n", deadzone[0], deadzone[1], deadzone[2], deadzone[3], deadzone[4], deadzone[5]);
    }

    ret = robot.robot_set_cst_ftframe(LEFT, 1);
    ret = robot.robot_set_cst_ftframe(RIGHT, 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int frame[2] = {-1, -1};
    ret = robot.robot_get_cst_ftframe(LEFT, frame);
    ret = robot.robot_get_cst_ftframe(RIGHT, frame+1);
    if (ret != ERR_SUCC)
    {
        std::cout << "get ftframe failed" << std::endl;
    }
    else
    {
        printf("ftframe: %d %d\n", frame[0], frame[1]);
    }

    ret = robot.robot_set_cst_ftframe(LEFT, 0);
    ret = robot.robot_set_cst_ftframe(RIGHT, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ret = robot.robot_get_cst_ftframe(LEFT, frame);
    ret = robot.robot_get_cst_ftframe(RIGHT, frame+1);
    if (ret != ERR_SUCC)
    {
        std::cout << "get ftframe failed" << std::endl;
    }
    else
    {
        printf("ftframe: %d %d\n", frame[0], frame[1]);
    }

    ret = robot.robot_set_ftsensor_filter(1, 10.1);
    ret = robot.robot_set_ftsensor_filter(2, 10.2);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));    
    double filter[2] = {};
    ret = robot.robot_get_ftsensor_filter(1, filter);
    ret += robot.robot_get_ftsensor_filter(2, filter+1);
    if (ret != 0)
    {
        std::cout << "get ftfilter failed" << std::endl;
    }
    else
    {
        printf("ftfilter: %f %f\n", filter[0], filter[1]);
    }

    ret = robot.robot_set_ftsensor_filter(1, 1.1);
    ret = robot.robot_set_ftsensor_filter(2, 1.2);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));    
    ret = robot.robot_get_ftsensor_filter(1, filter);
    ret += robot.robot_get_ftsensor_filter(2, filter+1);
    if (ret != 0)
    {
        std::cout << "get ftfilter failed" << std::endl;
    }
    else
    {
        printf("ftfilter: %f %f\n", filter[0], filter[1]);
    }

    PayLoad payload[2] = {{1.1,{2.1,3.1,4.1},1},{2.1,{2.1,3.1,1.1},2}};
    ret = robot.robot_set_ftsensor_payload(1, *payload);
    ret = robot.robot_set_ftsensor_payload(2, *(payload+1));
    if (ret != 0)
    {
        std::cout << "set payload failed" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    PayLoad res[2] = {};
    ret = robot.robot_get_ftsensor_payload(LEFT, 1, res);
    ret = robot.robot_get_ftsensor_payload(RIGHT, 2, res+1);
    if (ret != 0)
    {
        std::cout << "get payload failed" << std::endl;
    }
    else
    {
         printf("left payload: %f %f %f %f\n", res[0].centroid.x, res[0].centroid.y, res[0].centroid.z, res[0].mass);
         printf("right payload: %f %f %f %f\n", res[1].centroid.x, res[1].centroid.y, res[1].centroid.z, res[1].mass);
    }

    PayLoad payload2[2] = {{5,{60,-60,30},1},{0,{0,0,0},2}};
    ret = robot.robot_set_ftsensor_payload(1, *payload2);
    ret = robot.robot_set_ftsensor_payload(2, *(payload2+1));
    if (ret != 0)
    {
        std::cout << "set payload failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ret = robot.robot_get_ftsensor_payload(LEFT, 1, res);
    ret = robot.robot_get_ftsensor_payload(RIGHT, 2, res+1);
    if (ret != 0)
    {
        std::cout << "get payload failed" << std::endl;
    }
    else
    {
         printf("left payload: %f %f %f %f\n", res[0].centroid.x, res[0].centroid.y, res[0].centroid.z, res[0].mass);
         printf("right payload: %f %f %f %f\n", res[1].centroid.x, res[1].centroid.y, res[1].centroid.z, res[1].mass);
    }

    ret = robot.robot_set_cst_ftconfig(LEFT, 0, 1, 10, 1, 1);
    ret = robot.robot_set_cst_ftconfig(LEFT, 1, 0, 10, 1, 2);
    ret = robot.robot_set_cst_ftconfig(LEFT, 2, 1, 10, 1, 3);
    ret = robot.robot_set_cst_ftconfig(LEFT, 3, 0, 10, 1, 0.1);
    ret = robot.robot_set_cst_ftconfig(LEFT, 4, 1, 10, 1, 0.1);
    ret = robot.robot_set_cst_ftconfig(LEFT, 5, 0, 10, 1, 0.1);

    ret = robot.robot_set_cst_ftconfig(RIGHT, 0, 0, 10, 1, 3);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 1, 1, 20, 2, 4);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 2, 0, 30, 3, 5);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 3, 1, 40, 4, 0.2);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 4, 0, 50, 5, 0.3);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 5, 1, 60, 6, 0.4);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RobotAdmitCtrl ftconfig[2];
    ret = robot.robot_get_cst_ftconfig(LEFT, ftconfig);
    ret = robot.robot_get_cst_ftconfig(RIGHT, ftconfig + 1);
    if (ret != 0) 
    {
        printf("robot_get_cst_ftconfig failed!\n");
    } 
    else 
    {
        printf("robot_get_cst_ftconfig success!\n");
        for (int i = 0; i < 6; i++)
        {
            printf("ftconfig[%d] = enable %d b %f k %f t %f\n", i, ftconfig[0].admit_ctrl[i].opt, ftconfig[0].admit_ctrl[i].ft_user, ftconfig[0].admit_ctrl[i].ft_rebound, ftconfig[0].admit_ctrl[i].ft_constant);
        }
        for (int i = 0; i < 6; i++)
        {
            printf("ftconfig[%d] = enable %d b %f k %f t %f\n", i, ftconfig[1].admit_ctrl[i].opt, ftconfig[1].admit_ctrl[i].ft_user, ftconfig[1].admit_ctrl[i].ft_rebound, ftconfig[1].admit_ctrl[i].ft_constant);
        }
    }

    robot.robot_enable_force_control(LEFT);
    robot.robot_enable_force_control(RIGHT);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int stat[2] = {-1, -1};
    robot.robot_get_force_control_stat(LEFT, stat);
    robot.robot_get_force_control_stat(RIGHT, stat + 1);
    printf("Got for state: %d %d\n", stat[0], stat[1]);


    for (int i = 0; i < 100; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int status[2] = {-3, -3};
        int errorcode[2] = {-1, -1};
        double ft_original[6] = {};
        double ft_actual[6] = {};
        ret = robot.robot_get_ftsensor_stat(1, status, errorcode, ft_original, ft_actual);
        if (ret != 0)
        {
            printf("robot_get_ftsensor_stat failed!\n");
        }
        else
        {
            printf("ft_original: %f %f %f %f %f %f\n", ft_original[0], ft_original[1], ft_original[2], ft_original[3], ft_original[4], ft_original[5]);
            printf("ft_actual: %f %f %f %f %f %f\n", ft_actual[0], ft_actual[1], ft_actual[2], ft_actual[3], ft_actual[4], ft_actual[5]);
        }
        ret = robot.robot_get_ftsensor_stat(2, status + 1, errorcode + 1, ft_original, ft_actual);
        if (ret != 0)
        {
            printf("robot_get_ftsensor_stat failed!\n");
        }
        else
        {
            printf("ft_original: %f %f %f %f %f %f\n", ft_original[0], ft_original[1], ft_original[2], ft_original[3], ft_original[4], ft_original[5]);
            printf("ft_actual: %f %f %f %f %f %f\n", ft_actual[0], ft_actual[1], ft_actual[2], ft_actual[3], ft_actual[4], ft_actual[5]);
        }

        printf("status: %d %d\n", status[0], status[1]);
        printf("errorcode: %d %d\n", errorcode[0], errorcode[1]);
    }


    robot.robot_disable_force_control(LEFT);
    robot.robot_disable_force_control(RIGHT);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    robot.robot_get_force_control_stat(LEFT, stat);
    robot.robot_get_force_control_stat(RIGHT, stat + 1);
    printf("Got for state: %d %d\n", stat[0], stat[1]);

    robot.login_out();
    return 0;
}
