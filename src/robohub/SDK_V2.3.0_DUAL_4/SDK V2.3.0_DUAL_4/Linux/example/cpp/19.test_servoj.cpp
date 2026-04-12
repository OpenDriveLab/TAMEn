#include <JAKAZuRobot.h>
#include <iostream>
#include <thread>
#include <chrono>

#if defined(_WIN32)
#include <windows.h>
#endif

std::ostream& operator<<(std::ostream & os, const JointValue& jpos)
{
    os << "[";
    for (size_t i = 0; i < 6; i++)
    {
        os << jpos.jVal[i] << ",";
    }
    os << "]";
    return os;
}

int main()
{
    JAKAZuRobot::static_Set_SDK_filepath("./jakaLog.txt");

#if defined(_WIN32)
    // Get the current process handle
    HANDLE hProcess = GetCurrentProcess();

    // Raise the current process priority
    if (!SetPriorityClass(hProcess, HIGH_PRIORITY_CLASS)) {
        std::cerr << "eeeeeeee" << GetLastError() << std::endl;
        return 1;
    }
#endif

    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");
    robot.power_on();
    robot.enable_robot();

    JointValue jpos;
    robot.get_joint_position(&jpos);

    JointValue first_jpos = jpos, second_jpos = jpos;
    second_jpos.jVal[5] += 3.1415926 * 0.008;

    std::cout << "f: " << first_jpos << "\n";
    std::cout << "s: " << second_jpos << "\n";

    int i = 0;
    robot.servo_move_enable(1,1,0);
    auto starttime = std::chrono::steady_clock::now();
    while (i++ < 1000)
    {
        // std::cout << "st: ------------------- \n";
        robot.servo_j(&first_jpos, MoveMode::ABS);
        // std::cout << "md: ------------------- \n";
        robot.servo_j(&second_jpos, MoveMode::ABS);
        // std::cout << "ed: ------------------- \n";
    }
    auto endtime = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(endtime - starttime);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    std::cout << "use time: " << dur.count() << "\n";
    robot.servo_move_enable(0,1,0);
    robot.login_out();

    return 0;
}