#include <JAKAZuRobot.h>
#include <iostream>

#define PI 3.141592654

void c_mov(JAKAZuRobot& robot)
{
        //     end_p.tran.x = -506.468; end_p.tran.y = 58.250; end_p.tran.z = 397.879;
    // end_p.rpy.rx = 1.479; end_p.rpy.ry = - -0.786; end_p.rpy.rz = -1.560;

    // mid_p.tran.x = -506.468; mid_p.tran.y = 14.102; mid_p.tran.z = 397.879;
    // mid_p.rpy.rx = 1.479; mid_p.rpy.ry = -0.786; mid_p.rpy.rz = -1.560;

    // JKTYPE.JointValue joint_pos = new JKTYPE.JointValue();
    // joint_pos.jVal = new double[] { (152.245 * 0.01745),  (93.330 * 0.01745), (-89.587 * 0.01745), (0.296 * 0.01745), (-114.570 * 0.01745), (46.883 * 0.01745) };
    // jakaAPI.joint_move(ref handle1, ref joint_pos, JKTYPE.MoveMode.ABS, true, 0.2);
    robot.motion_abort();

    JointValue jpos = {152.216 * 0.01745, 93.312 * 0.01745, -89.570 * 0.01745, 0.296 * 0.0174, -114.548 * 0.01745, 46.874 * 0.01745};
    // JointValue jpos = {0, 90 * PI/ 180.0, 90 * PI/ 180.0, 0 * PI/ 180.0, -90 * PI/ 180.0, 0};
    robot.joint_move(&jpos, MoveMode::ABS, TRUE, PI, 10, 0, NULL);

    CartesianPose cp1, cp2;
    // cp1.tran.x = -253.016;
    // cp1.tran.y = 12.0;
    // cp1.tran.z = 514.05;
    // cp1.rpy.rx = -90.0 / 180 * PI;
    // cp1.rpy.ry = 0.0 / 180 * PI;
    // cp1.rpy.rz = 90.0 / 180 * PI;

    // CartesianPose cp1, cp2;
    // cp1.tran.x = -506.468;
    // cp1.tran.y = 58.250;
    // cp1.tran.z = 397.879;
    // cp1.rpy.rx = 1.479;
    // cp1.rpy.ry = 0.0 / 180 * PI;
    // cp1.rpy.rz = 90.0 / 180 * PI;

    cp1.tran.x = -506.468;
    cp1.tran.y = 14.102;
    cp1.tran.z = 397.879;
    cp1.rpy.rx = 84.798 / 180 * PI;
    cp1.rpy.ry = -45.069 / 180 * PI;
    cp1.rpy.rz = -89.544 / 180 * PI;

    cp2.tran.x = -506.468;
    cp2.tran.y = 58.25;
    cp2.tran.z = 397.879;
    cp2.rpy.rx = 84.798 / 180 * PI;
    cp2.rpy.ry = -45.069 / 180 * PI;
    cp2.rpy.rz = -89.544 / 180 * PI;

    // cp2.tran.x = -381.435;
    // cp2.tran.y = 0.744;
    // cp2.tran.z = 514.05;
    // cp2.rpy.rx = -90.0 / 180 * PI;
    // cp2.rpy.ry = 0.0 / 180 * PI;
    // cp2.rpy.rz = 90.0 / 180 * PI;
    
    for (int i = 0; i < 5; i++)
    {
        robot.circular_move(&cp1, &cp2, MoveMode::ABS, TRUE, 30, 572, 0.1, NULL, 1);
    }

    while (true)
    {
        BOOL flag;
        robot.is_in_pos(&flag);
        if (flag)
        {
            std::cout << "exit c mov!\n";
            break;
        }
    }
}

void prog_c_mov(JAKAZuRobot &robot)
{
    robot.program_load("New Program");
    robot.program_run();

    while (1)
    {
        ProgramState state;
        robot.get_program_state(&state);
        if (state == PROGRAM_IDLE)
        {
            std::cout << "exit prog c mov!\n";
            break;
        }
    }
}

int main()
{
    JAKAZuRobot::static_Set_SDK_filepath("./jakaLog.txt");

    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");
    robot.power_on();
    robot.enable_robot();

    int i = 0;
    // while (true)
    {
        // if (i%2)
        // {
        //     prog_c_mov(robot);
        // }
        // else
        // {
            c_mov(robot);
        // }
        // i++;
    }

    robot.login_out();
    return 0;
}