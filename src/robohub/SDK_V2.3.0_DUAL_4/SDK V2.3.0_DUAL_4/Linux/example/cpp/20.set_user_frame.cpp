#include <JAKAZuRobot.h>
#include <cassert>
#include <iostream>
#include <chrono>
#include <thread>

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
    JAKAZuRobot::static_Set_SDK_filepath("./jakaLog.txt");

    const int id = 1;
    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");

    while (true)
    {
        CartesianPose pose { 1, 2, 3, 5 ,6, 5};

        if (robot.set_user_frame_data(id, &pose, "1234") != ERR_SUCC)
        {
            std::cout << "set_user_frame_data error !\n";
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        CartesianPose pose_feedback;
        if (robot.get_user_frame_data(id, &pose_feedback) != ERR_SUCC)
        {
            std::cout << "get_user_frame_data error !\n";
        }

        std::cout << "after upos: " << pose_feedback << "\n";
    }

    assert(robot.login_out() == ERR_SUCC);
    return 0;
}