#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include "timespec.h"
#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)

void servop_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    jpos[0].jVal[0] = deg2rad(-74);
    jpos[0].jVal[1] = deg2rad(-5);
    jpos[0].jVal[2] = deg2rad(67);
    jpos[0].jVal[3] = deg2rad(-51);
    jpos[0].jVal[4] = deg2rad(2);
    jpos[0].jVal[5] = deg2rad(-40);
    jpos[0].jVal[6] = deg2rad(-20);
 
    double jv[2] = {deg2rad(10)};
    double ja[2] = {deg2rad(100)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(LEFT,mode,true,jpos,jv,ja);

    double v[] = {10,10};
    double a[] = {100,100};
    CartesianPose cpos[2];
    robot.kine_forward(0,&jpos[0],&cpos[0]);
    robot.kine_inverse(0,&jpos[0],&cpos[0],&jpos[1]);
    cpos[0].tran.z += 10;
    robot.robot_run_multi_movl(LEFT,mode,true,cpos,v,a);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    robot.robot_zero_ftsensor(0);
    robot.robot_zero_ftsensor(1);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    //robot.robot_zero_ftsensor(0);
    //double deadzone[6] = {.1,.2,.3,.4,.5,.6};
    //robot.robot_set_ftsensor_deadzone(0, deadzone);
    robot.robot_set_cst_ftframe(LEFT, 0);
    //robot.robot_set_cst_ftframe(RIGHT, 1);
    //robot.robot_set_ftsensor_filter( 0, 10.1);
    //robot.robot_set_ftsensor_filter(0, 10.2);
    //PayLoad payload[2] = {{0,{2,3,4}},{0,{2,3,1}}};
    //robot.robot_set_ftsensor_payload(LEFT, 0, *payload);
    //robot.robot_set_ftsensor_payload(RIGHT, 1, *(payload+1));
    robot.robot_set_cst_ftconfig(LEFT, 0, 0, 100, 1, 0);
    robot.robot_set_cst_ftconfig(LEFT, 1, 0, 100, 1, 0);
    robot.robot_set_cst_ftconfig(LEFT, 2, 1, 20, 1, 0);
    robot.robot_set_cst_ftconfig(LEFT, 3, 0, 10, 1, 0);
    robot.robot_set_cst_ftconfig(LEFT, 4, 0, 10, 1, 0);
    robot.robot_set_cst_ftconfig(LEFT, 5, 0, 10, 1, 0);
    // robot.robot_enable_force_control(RIGHT);
    robot.robot_enable_force_control(LEFT);

    memcpy(&cpos[1],&cpos[0],sizeof(cpos[0]));
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1,1, 0);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    
    for(int i=0;;i++)
    {
        robot.edg_recv(&next);

        JointValue actjpos;
        CartesianPose actcpos;
        robot.edg_get_stat(0, &actjpos, &actcpos);
        double coefficient = sin(i/1000.0);
        double z_step = 50 * coefficient;
        double x_step = 10 * coefficient;  
        cpos[1].tran.z = cpos[0].tran.z + z_step;
        cpos[1].tran.x = cpos[0].tran.x + x_step;
        double ori_step = deg2rad(10) * coefficient;
        cpos[1].rpy.rx = cpos[0].rpy.rx + ori_step;
        cpos[1].rpy.ry = cpos[0].rpy.ry + ori_step;
        cpos[1].rpy.rz = cpos[0].rpy.rz + ori_step;
        robot.edg_servo_p(0,&cpos[1],MoveMode::ABS);
        robot.edg_send();
        printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", actcpos.tran.x, actcpos.tran.y, actcpos.tran.z, actcpos.rpy.rx, actcpos.rpy.ry, actcpos.rpy.rz);


        // Wait for the next cycle
        timespec dt;
        dt.tv_nsec = 1000000;
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }
    robot.robot_disable_force_control(LEFT);
}

int main()
{
    //
    JAKAZuRobot robot;

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");
    // robot.login_in("10.5.5.100");

    robot.servo_move_enable(0,1, 0);
    robot.servo_move_enable(0,1, 1); // Disable servo mode for all robots
    robot.servo_move_use_none_filter();
    robot.motion_abort();
    robot.power_on();
    robot.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    robot_edg_init(1,"192.168.2.255"); // edg_init must be called before using EDG APIs

    // servoj_test(robot);
    servop_test(robot);
    
    robot.login_out();
    return 0;
}