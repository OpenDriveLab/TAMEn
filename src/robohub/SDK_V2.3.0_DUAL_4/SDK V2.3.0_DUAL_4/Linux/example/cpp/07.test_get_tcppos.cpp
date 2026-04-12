#include "jkerr.h"
#include "jktypes.h"
#include <JAKAZuRobot.h>
#include <cassert>
#include <iostream>
#include <ostream>

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

int main()
{
    JAKAZuRobot robot;
    CartesianPose pose;
    errno_t ret = ERR_SUCC;

    ret = robot.login_in("192.168.2.200");
    assert(ret == ERR_SUCC);

    ret = robot.power_on();
    assert(ret == ERR_SUCC);

    ret = robot.enable_robot();
    assert(ret == ERR_SUCC);

    while (true)
    {
        std::cout << "=======================" << std::endl;
        ret = robot.get_tcp_position(&pose);
        std::cout << "get_tcp_position: " << (ret == ERR_SUCC ? "SUCC" : "FAIL") << "\n"
            << pose << std::endl;
    }

    robot.login_out();
    return 0;
}
