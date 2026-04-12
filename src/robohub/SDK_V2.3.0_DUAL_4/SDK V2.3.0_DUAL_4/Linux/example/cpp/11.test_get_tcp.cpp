#include <JAKAZuRobot.h>
#include <chrono>
#include <thread>

int main()
{
    JAKAZuRobot robot;
    CartesianPose tcp_pos;
    errno_t ret;
    robot.login_in("192.168.2.200");
    robot.power_on();
    robot.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    while (true)
    {
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
        }

        ret = robot.jog_stop(0);
        if (ret != ERR_SUCC)
        {
            printf("jog_stop failed.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            printf("jog_stop successfully.\n");
        }
        // printf("%lf, %lf, %lf, %lf, %lf, %lf",
        //    tcp_pos.tran.x, tcp_pos.tran.y, tcp_pos.tran.z,
        //    tcp_pos.rpy.rx, tcp_pos.rpy.ry, tcp_pos.rpy.rz);
    }
    robot.login_out();
    return 0;
}
