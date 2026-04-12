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
    errno_t ret = robot.login_in("192.168.2.200");

    PayLoad left = {1,2,3,4};
    PayLoad right = {5,6,7,8};
    ret = robot.robot_set_tool_payload(0, &left);
    if (ret != ERR_SUCC)
    {
        printf("set left payload failed\n");
    } 
    ret = robot.robot_set_tool_payload(1, &right);
    if (ret != ERR_SUCC)
    {
        printf("set right payload failed\n");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    PayLoad res[2];
    ret = robot.robot_get_tool_payload(res);
    if (ret != ERR_SUCC)
    {
        printf("get right payload failed\n");
    }
    printf("left: %f %f %f %f\n", res[0].centroid.x, res[0].centroid.y, res[0].centroid.z, res[0].mass);
    printf("right: %f %f %f %f\n", res[1].centroid.x, res[1].centroid.y, res[1].centroid.z, res[1].mass);

    robot.login_out();
    return 0;
}
