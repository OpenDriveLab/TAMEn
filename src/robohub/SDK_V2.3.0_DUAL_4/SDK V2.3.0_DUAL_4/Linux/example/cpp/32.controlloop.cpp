#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)

int main()
{
    //
    JAKAZuRobot robot;

    // robot.login_in("127.0.0.1");
    robot.login_in("192.168.2.200");
    robot.power_on(); 
    // robot.enable_robot();
    
    robot.set_joint_controlloop(0,4,60,100,4000);
#if 1
    std::thread t([&](){
        bool en_loop = false;
        robot.enable_joint_controlloop(0,5,0);
        robot.enable_joint_controlloop(0,4,0);
        robot.enable_joint_controlloop(0,6,0);

        robot.enable_joint_controlloop(0,4,1);
        // robot.enable_joint_controlloop(0,4,1);
        // robot.enable_joint_controlloop(0,6,1);
        for(int i =0;;i++)
        {
            printf("i = %d\n",i);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            // robot.set_joint_controlloop(0,5,1,100,4000);
            robot.set_joint_controlloop(0,4,1,100,4000);
            // robot.set_joint_controlloop(0,6,1,100,4000);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            // robot.set_joint_controlloop(0,5,99,50,2000);
            robot.set_joint_controlloop(0,4,99,50,2000);
            // robot.set_joint_controlloop(0,6,99,50,2000);
        }
    });
#endif
    while(1){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    for(int i = 0;;i++)
    {
        JointValue jpos[2];
        memset(jpos,0,sizeof(jpos));
        double v[2]={deg2rad(90),deg2rad(90)};
        double acc[2]={deg2rad(200),deg2rad(200)};
        MoveMode mode[2]={MoveMode::ABS,MoveMode::ABS};
        robot.robot_run_multi_movj(DUAL,mode,true,jpos,v,acc);

        JointValue jpos2[2];
        memset(jpos2,0,sizeof(jpos2));
        jpos2[0].jVal[6]=deg2rad(120);
        jpos2[1].jVal[6]=deg2rad(120);
        robot.robot_run_multi_movj(DUAL,mode,true,jpos2,v,acc);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    robot.enable_joint_controlloop(0,6,0);

    robot.login_out();
    return 0;
}