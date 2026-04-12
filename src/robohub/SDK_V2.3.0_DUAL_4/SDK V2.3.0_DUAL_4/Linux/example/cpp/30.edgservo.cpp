#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include "timespec.h"

#define CONTROL_LOOP_MS 1

#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)
int64_t edg_timespec2ns(timespec t)
{
    return t.tv_sec * 1000000000 + t.tv_nsec;
}
void edg_sync(timespec reftime_, int64_t *sys_time_offset)
{
    auto reftime = edg_timespec2ns(reftime_);
    static int64_t integral = 0;
    int64_t cycletime = 1000000;
    int64_t delta = (reftime - 0) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    if (sys_time_offset)
    {
        *sys_time_offset = -(delta / 100) - (integral / 20);  // Similar to PI adjustment
    }
}

int servoj_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    
    double v[] = {deg2rad(30),deg2rad(30)};
    double a[] = {deg2rad(150),deg2rad(150)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,v,a);
    robot.servo_move_use_none_filter();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    JointValue jpos_cmd;
    memset(&jpos_cmd,0,sizeof(jpos_cmd));
    JointValue jpos_cmd2;
    memset(&jpos_cmd2,0,sizeof(jpos_cmd2));

    bool rob1_change_to_servo = false;
    
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1,1, 0);
    robot.servo_move_enable(1,1, 1);
    auto next_time = std::chrono::steady_clock::now();
    
    int rob2_start_k = 0;
    uint32_t cmd_index = 0;
    for(int i = 0;;i++)
    {
        JointValue jpos[2];
        CartesianPose cpos[2];
        robot.edg_get_stat(0, &jpos[0], &cpos[0]);
        robot.edg_get_stat(1, &jpos[1], &cpos[1]);
    
        double t = (i - 0)/10000.0;
        double kk = 35;
        jpos_cmd.jVal[0] = sin(kk*t)*30/180.0*3.14;
        jpos_cmd.jVal[1] = -cos(kk*t)*20 /180.0*3.14 + 20/180.0*3.14;
        jpos_cmd.jVal[3] = -cos(kk*t)*10/180.0*3.14 + 10/180.0*3.14;
        robot.edg_servo_j(0, &jpos_cmd, MoveMode::ABS);
        robot.edg_servo_j(1, &jpos_cmd, MoveMode::ABS);
        robot.edg_send(&cmd_index);
        cmd_index++;
#if 1
        printf("%012ld:\t %010d\t left_stat=[%0.4f %0.4f %0.4f] right_stat=[%0.4f %0.4f %0.4f] cmd=[%0.4f %0.4f %0.4f]\n", 
        std::chrono::duration_cast<std::chrono::nanoseconds>(next_time.time_since_epoch()).count(),
        cmd_index,
        jpos[0].jVal[0], jpos[0].jVal[1], jpos[0].jVal[3],
        jpos[1].jVal[0], jpos[1].jVal[1], jpos[1].jVal[3],
        jpos_cmd.jVal[0], jpos_cmd.jVal[1], jpos_cmd.jVal[3]
        );
#endif
        // Wait for the next cycle
        next_time += std::chrono::milliseconds(CONTROL_LOOP_MS);
        std::this_thread::sleep_until(next_time);
    }
}

void servop_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    jpos[0].jVal[0] = deg2rad(28);
    jpos[0].jVal[1] = deg2rad(33);
    jpos[0].jVal[2] = deg2rad(-14);
    jpos[0].jVal[3] = deg2rad(-55);
    jpos[0].jVal[4] = deg2rad(-24);
    jpos[0].jVal[5] = deg2rad(-53);
    jpos[0].jVal[6] = deg2rad(40);

    jpos[1].jVal[0] = deg2rad(-27);
    jpos[1].jVal[1] = deg2rad(50);
    jpos[1].jVal[2] = deg2rad(11);
    jpos[1].jVal[3] = deg2rad(-75);
    jpos[1].jVal[4] = deg2rad(18);
    jpos[1].jVal[5] = deg2rad(30);
    jpos[1].jVal[6] = deg2rad(25);
 
    double jv[2] = {deg2rad(10),deg2rad(10)};
    double ja[2] = {deg2rad(100),deg2rad(100)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    // Before motion, ensure the pose is valid and reachable
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,jv,ja);

    double v[] = {10,10};
    double a[] = {100,100};
    CartesianPose cpos[2];
    robot.kine_forward(0,&jpos[0],&cpos[0]);
    robot.kine_forward(1,&jpos[1],&cpos[1]);
    cpos[0].tran.z += 10;
    cpos[1].tran.z += 10;
    robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1,1, 0);
    robot.servo_move_enable(1,1, 1);
    auto next_time = std::chrono::steady_clock::now();
    CartesianPose servo_cpos[2];
    memcpy(servo_cpos,cpos,sizeof(servo_cpos));
    uint32_t cmd_index = 0;
    for(int i=0;;i++)
    {

        JointValue actjpos[2];
        CartesianPose actcpos[2];

        robot.edg_get_stat(0, &actjpos[0], &actcpos[0]);
        robot.edg_get_stat(1, &actjpos[1], &actcpos[1]);

        double coefficient = sin(i/300.0);
        double z_step = 100 * coefficient;
        double x_step = 10 * coefficient;  
        double ori_step = deg2rad(10) * coefficient;
  
        servo_cpos[0].tran.z = cpos[0].tran.z + z_step;
        servo_cpos[0].tran.x = cpos[0].tran.x + x_step;
        servo_cpos[1].tran.z = cpos[1].tran.z + z_step;
        servo_cpos[1].tran.x = cpos[1].tran.x + x_step;
        robot.edg_servo_p(0,&servo_cpos[0],MoveMode::ABS);
        robot.edg_servo_p(1,&servo_cpos[1],MoveMode::ABS);
        robot.edg_send();
        cmd_index++;
#if 1
        printf("%012ld:\t %010d\t left_stat=[%0.4f %0.4f %0.4f] right_stat=[%0.4f %0.4f %0.4f] left_cmd=[%0.4f %0.4f %0.4f] right_cmd=[%0.4f %0.4f %0.4f]\n", 
        std::chrono::duration_cast<std::chrono::nanoseconds>(next_time.time_since_epoch()).count(),
        cmd_index,
        actcpos[0].tran.x, actcpos[0].tran.y, actcpos[0].tran.z,
        actcpos[1].tran.x, actcpos[1].tran.y, actcpos[1].tran.z,
        servo_cpos[0].tran.x, servo_cpos[0].tran.y, servo_cpos[0].tran.z,
        servo_cpos[1].tran.x, servo_cpos[1].tran.y, servo_cpos[1].tran.z
        );
#endif

        // Wait for the next cycle
        next_time += std::chrono::milliseconds(8);
        std::this_thread::sleep_until(next_time);
    }
    robot.servo_move_enable(0,1, 0);
    robot.servo_move_enable(0,1, 1);
}

void test_edg_stat(JAKAZuRobot &robot)
{
    bool stop=false;
    auto stat_thread = std::thread([&](){
        while(!stop)
        {
            JointValue jpos[2];
            CartesianPose cpos[2];
            CartesianPose sensor_torque[2];
            
            robot.edg_get_stat(0, &jpos[0], &cpos[0], &sensor_torque[0]);
            robot.edg_get_stat(1, &jpos[1], &cpos[1], &sensor_torque[1]);
            
            unsigned long int details[3];
            robot.edg_stat_details(details);

            auto now = std::chrono::steady_clock::now();
            auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

            bool keys[2];
            robot.edg_stat_free_key(keys);
            printf("EDG Stat Time:%012ld:\n", ns);
            printf("\tkeys[0]=%d, keys[1]=%d\n", keys[0], keys[1]);
            printf("\tdetails[0]=%ld, details[1]=%ld, details[2]=%ld\n", details[0], details[1], details[2]);
            printf("\tRB0:%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", jpos[0].jVal[0], jpos[0].jVal[1], jpos[0].jVal[2], jpos[0].jVal[3], jpos[0].jVal[4], jpos[0].jVal[5]);
            printf("\tRB1:%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", jpos[1].jVal[0], jpos[1].jVal[1], jpos[1].jVal[2], jpos[1].jVal[3], jpos[1].jVal[4], jpos[1].jVal[5]);
            printf("\tRB0_cpos:%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", cpos[0].tran.x, cpos[0].tran.y, cpos[0].tran.z, cpos[0].rpy.rx, cpos[0].rpy.ry, cpos[0].rpy.rz);
            printf("\tRB1_cpos:%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", cpos[1].tran.x, cpos[1].tran.y, cpos[1].tran.z, cpos[1].rpy.rx, cpos[1].rpy.ry, cpos[1].rpy.rz);
            printf("\tRB0_sensor_torque:%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", sensor_torque[0].tran.x, sensor_torque[0].tran.y, sensor_torque[0].tran.z, sensor_torque[0].rpy.rx, sensor_torque[0].rpy.ry, sensor_torque[0].rpy.rz);
            printf("\tRB1_sensor_torque:%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", sensor_torque[1].tran.x, sensor_torque[1].tran.y, sensor_torque[1].tran.z, sensor_torque[1].rpy.rx, sensor_torque[1].rpy.ry, sensor_torque[1].rpy.rz);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    std::this_thread::sleep_for(std::chrono::seconds(30));

    // JointValue jpos[2];
    // JointValue jpos_end[2];
    // memset(&jpos,0,sizeof(jpos));
    // memset(&jpos_end,0,sizeof(jpos_end));
    // jpos[0].jVal[0] = deg2rad(28);
    // jpos[0].jVal[1] = deg2rad(33);
    // jpos[0].jVal[2] = deg2rad(-14);
    // jpos[0].jVal[3] = deg2rad(-55);
    // jpos[0].jVal[4] = deg2rad(-24);
    // jpos[0].jVal[5] = deg2rad(-53);
    // jpos[0].jVal[6] = deg2rad(40);

    // jpos[1].jVal[0] = deg2rad(-27);
    // jpos[1].jVal[1] = deg2rad(50);
    // jpos[1].jVal[2] = deg2rad(11);
    // jpos[1].jVal[3] = deg2rad(-75);
    // jpos[1].jVal[4] = deg2rad(18);
    // jpos[1].jVal[5] = deg2rad(30);
    // jpos[1].jVal[6] = deg2rad(25);
 
    // double jv[2] = {deg2rad(10),deg2rad(10)};
    // double ja[2] = {deg2rad(100),deg2rad(100)};
    // MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    // robot.robot_run_multi_movj(DUAL,mode,true,jpos,jv,ja);
    // robot.robot_run_multi_movj(DUAL,mode,true,jpos_end,jv,ja);

    stop = true;
    stat_thread.join();
}

int main()
{
    //
    JAKAZuRobot robot;

    robot.login_in("192.168.2.200");
    robot.clear_error();
    robot.servo_move_enable(0,1, 0);
    robot.servo_move_enable(0,1, 1); // Disable servo mode for all robots
    robot.servo_move_use_none_filter();
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();

    robot_edg_init(1,"192.168.2.255"); // edg_init must be called before using EDG APIs

    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Test reading robot state
    test_edg_stat(robot);
    
    // Test joint servo; CONTROL_LOOP_MS is 8 for CAN dual-arm and 1 for ECAT dual-arm
    // servoj_test(robot);
    
    // Test Cartesian servo; CONTROL_LOOP_MS is 8 for CAN dual-arm and 1 for ECAT dual-arm
    // servop_test(robot);
    
    robot.login_out();
    return 0;
}