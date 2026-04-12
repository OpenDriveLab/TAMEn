#include <JAKAZuRobot.h>
#include <algorithm>
#include <iostream>
#include <type_traits>

template <int n>
std::ostream& operator<<(std::ostream & os, const JointMonitorData (&arr)[n])
{
    os << "[";
    for (size_t i = 0; i < n; i++)
    {
        os << arr[i].instVel << ",";
    }
    os << "]";
    return os;
}

int main()
{
    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");

    RobotStatus rbs;
    errno_t eret = robot.get_robot_status(&rbs);
    int i = 10000000;
    while (i--)
        if (eret == ERR_SUCC)
        {
            std::cout << "torq sensor ip: " << rbs.torq_sensor_monitor_data.ip << std::endl;
            std::cout << "joint monitor data: " << rbs.robot_monitor_data.jointMonitorData << std::endl;
            std::cout << "is socket connect: " << rbs.is_socket_connect << std::endl;
        }

    robot.login_out();
    return 0;
}