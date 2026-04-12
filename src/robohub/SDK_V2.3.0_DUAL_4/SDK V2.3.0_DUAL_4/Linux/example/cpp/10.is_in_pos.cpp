#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <ostream>
#include <thread>

#define JK_PI (3.141592653589793)

std::ostream& operator<<(std::ostream& out, const Rpy& rpy)
{
    return out << "{ rx: " << rpy.rx << ", ry: " << rpy.ry << ", rz: " << rpy.rz << "}";
}

std::ostream& operator<<(std::ostream& out, const CartesianTran& tran)
{
    return out << "{ x: " << tran.x << ", y: " << tran.y << ", z: " << tran.z << "}";
}

std::ostream& operator<<(std::ostream& out, const CartesianPose& pos)
{
    return out << "{\n"
        << "tran: " << pos.tran << ",\n"
        << "rpy" << pos.rpy << "\n"
        << "}";
}

std::ostream& operator<<(std::ostream& out, const JointValue& jval)
{
    out << '{';
    for (int i = 0; i < 6; i++)
    {
        out << jval.jVal[i] << ',';
    }
    return out << '}';
}

int main()
{
    JAKAZuRobot robot;
    errno_t ret;
    JointValue cur_jval {0};
    BOOL inpos = 0;
    BOOL last_inpos = 0;
    double thresholding;
    
    ret = robot.login_in("192.168.2.200");
    if(ret != ERR_SUCC)
    {
        std::cout << "login failed.\n";
        exit(EXIT_FAILURE);
    }

    ret = robot.power_on();
    if(ret != ERR_SUCC)
    {
        std::cout << "power on failed.\n";
        exit(EXIT_FAILURE);
    }

    ret = robot.enable_robot();
    if(ret != ERR_SUCC)
    {
        std::cout << "enable robot failed.\n";
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        robot.get_joint_position(&cur_jval);
        robot.is_in_pos(&inpos);
        if (inpos != last_inpos)
            std::cout << "jval is " << cur_jval << "{" <<  (inpos ? "True" : "False") << ","
                << (last_inpos ? "True" : "False") << "}" << '\n';
        last_inpos = inpos;
    }

    robot.login_out();
    return 0;
}
