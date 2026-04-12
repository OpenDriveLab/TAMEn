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
    JointValue start_pos[2] = { { 0, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 0, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };

    CartesianPose c_pos[2] { { 0, 0, 30, 0, 0, 0 }, { 0, 0, 60, 0, 0, 0 } };
    
    ret = robot.login_in("10.5.5.100");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    robot.clear_error();

    robot.set_collision_level(LEFT, 3);
    robot.set_collision_level(RIGHT, 4);

    int level[2] = {-1, -1};
    robot.get_collision_level(LEFT, level);
    robot.get_collision_level(RIGHT, level+1);
    printf("left level: %d, right level: %d\n", level[0], level[1]);

    robot.set_debug_mode(true);

    robot.set_network_exception_handle(1500, ProcessType::MOT_ABORT);

    robot.set_block_wait_timeout(20);   // Maximum allowed blocking time: 20 s

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    {
        MoveMode moveop[2] = {ABS, ABS};
        double vel[2] = {.5, .5};
        double acc[2] = {1, 1};
        ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, start_pos, vel, acc);    // Block and wait up to 20 s
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

        ret = robot.robot_run_multi_movj(RIGHT, moveop, TRUE, start_pos, vel, acc);    // Block and wait up to 20 s
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

    }

    {
        MoveMode moveop[2] = {INCR, INCR};
        double vel[2] = {10, 10};
        double acc[2] = {10, 10};
        ret = robot.robot_run_multi_movl(LEFT, moveop, TRUE, c_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
             printf("Error happends when move to cart. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear_move pos ok.\n";
        }

        ret = robot.robot_run_multi_movl(RIGHT, moveop, TRUE, c_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
             printf("Error happends when move to cart. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear_move pos ok.\n";
        }
    }

    {
        MoveMode moveop[2] = {ABS, ABS};
        double vel[2] = {.5, .5};
        double acc[2] = {1, 1};
        ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, start_pos, vel, acc);    // Block and wait up to 20 s
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

        ret = robot.robot_run_multi_movj(RIGHT, moveop, TRUE, start_pos, vel, acc);    // Block and wait up to 20 s
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

    }

    {
        MoveMode moveop[2] = {INCR, INCR};
        double vel[2] = {10, 10};
        double acc[2] = {10, 10};
        ret = robot.robot_run_multi_movl(DUAL, moveop, TRUE, c_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to cart. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear_move pos ok.\n";
        }

        ret = robot.robot_run_multi_movl(DUAL, moveop, TRUE, c_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to cart. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear_move pos ok.\n";
        }
    }

    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int limit = -1;
        ret = robot.robot_is_on_soft_limit(LEFT, &limit);
        printf("left limit: %d\n", limit);
        ret = robot.robot_is_on_soft_limit(RIGHT, &limit);
        printf("right limit: %d\n", limit);
        ret = robot.robot_is_on_soft_limit(DUAL, &limit);
        printf("dual limit: %d\n", limit);
        

        ErrorCode code;
        ret = robot.get_last_error(&code);
        printf("errcode = %lx, %s\n", code.code, code.message);
    }

    robot.login_out();
    return 0;
}
