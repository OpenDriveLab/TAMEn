#include <JAKAZuRobot.h>
#include <chrono>
#include <thread>

#define PI 3.1415926

int main()
{
    JAKAZuRobot robot;
    CartesianPose tcp_pos;
    errno_t ret;
    robot.login_in("192.168.2.200");
    robot.power_on();
    robot.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    JointValue jointPos1, jointPos2;
    jointPos1.jVal[0] = 0.0 * PI / 180;
    jointPos1.jVal[1] = 90.0 * PI / 180;
    jointPos1.jVal[2] = 90.0 * PI / 180;
    jointPos1.jVal[3] = 90.0 * PI / 180;
    jointPos1.jVal[4] = -90 * PI / 180;
    jointPos1.jVal[5] = 0 * PI / 180;

    jointPos2.jVal[0] = 0.0 * PI / 180;
    jointPos2.jVal[1] = 45.0 * PI / 180;
    jointPos2.jVal[2] = 45.0 * PI / 180;
    jointPos2.jVal[3] = 90.0 * PI / 180;
    jointPos2.jVal[4] = -90 * PI / 180;
    jointPos2.jVal[5] = 0 * PI / 180;

    while (true)
    {
        ret = robot.joint_move(&jointPos1, ABS, TRUE, 1, 0.05, 1, NULL);
        if (ret != ERR_SUCC)
        {
            printf("joint_move 1 failed.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        ret = robot.get_tcp_position(&tcp_pos);
        if (ret != ERR_SUCC)
        {
            printf("get_tcp_position failed.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            printf("get_tcp_position successfully.\n");
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            printf("%lf, %lf, %lf, %lf, %lf, %lf",
                tcp_pos.tran.x, tcp_pos.tran.y, tcp_pos.tran.z,
                tcp_pos.rpy.rx, tcp_pos.rpy.ry, tcp_pos.rpy.rz);
        }

        ret = robot.joint_move(&jointPos2, ABS, TRUE, 1, 0.05, 1, NULL);
        if (ret != ERR_SUCC)
        {
            printf("joint_move 2 failed.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        ret = robot.get_tcp_position(&tcp_pos);
        if (ret != ERR_SUCC)
        {
            printf("get_tcp_position failed.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            printf("get_tcp_position successfully.\n");
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            printf("%lf, %lf, %lf, %lf, %lf, %lf",
                tcp_pos.tran.x, tcp_pos.tran.y, tcp_pos.tran.z,
                tcp_pos.rpy.rx, tcp_pos.rpy.ry, tcp_pos.rpy.rz);
        }
    }
    // robot.login_out();
    return 0;
}
