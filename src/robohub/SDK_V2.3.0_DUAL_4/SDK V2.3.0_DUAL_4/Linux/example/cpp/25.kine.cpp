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
    RobotStatus robotStatus;
    errno_t ret;
    
    ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");
    // Before solving IK, ensure the pose is valid and reachable
    JointValue start_pos[2] = { { 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad},
                                 { 90 * deg_tp_rad, -45 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 90 * deg_tp_rad} };    
    CartesianPose pos[2];
    robot.kine_forward(LEFT, &start_pos[0], &pos[0]);
    robot.kine_forward(RIGHT, &start_pos[1], &pos[1]);
    printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
    printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

    JointValue end_pos[2];
    pos[0].tran.x += 20;
    pos[0].tran.y += 20;
    pos[0].tran.z += 20;
    pos[1].tran.x += 20;
    pos[1].tran.y += 20;
    pos[1].tran.z += 20;
    robot.kine_inverse(LEFT, &start_pos[0], &pos[0], &end_pos[0]);
    robot.kine_inverse(RIGHT, &start_pos[1], &pos[1], &end_pos[1]);

    printf("left end pos = %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[0].jVal[0], end_pos[0].jVal[1], end_pos[0].jVal[2], end_pos[0].jVal[3], end_pos[0].jVal[4], end_pos[0].jVal[5], end_pos[0].jVal[6]);
    printf("right end pos = %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[1].jVal[0], end_pos[1].jVal[1], end_pos[1].jVal[2], end_pos[1].jVal[3], end_pos[1].jVal[4], end_pos[1].jVal[5], end_pos[0].jVal[6]);

    robot.kine_forward(LEFT, &end_pos[0], &pos[0]);
    robot.kine_forward(RIGHT, &end_pos[1], &pos[1]);
    printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
    printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

    robot.login_out();
    return 0;
}
