#include <JAKAZuRobot.h>
#include <chrono>
#include <thread>
#include <iostream>

int main()
{
    JAKAZuRobot robot;
    errno_t ret;

    // Log in to the robot
    ret = robot.login_in("192.168.2.200");
    if (ret != ERR_SUCC) {
        std::cerr << "Login failed\n";
        return -1;
    }

    // Power on
    ret = robot.power_on();
    if (ret != ERR_SUCC) {
        std::cerr << "power_on failed\n";
    }

    // Clear errors
    robot.clear_error();

    // --- Configure TIO power ---
    ret = robot.set_tio_vout_param(0, 1, 0); // Left arm 24V
    if (ret != ERR_SUCC) std::cerr << "set_tio_vout_param left failed: " << ret << std::endl;

    ret = robot.set_tio_vout_param(1, 1, 0); // Right arm 24V
    if (ret != ERR_SUCC) std::cerr << "set_tio_vout_param right failed: " << ret << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3)); // Required: wait for hardware to stabilize

    // --- Read parameters ---
    for (int arm = 0; arm < 2; ++arm)
    {
        int enable = -1;
        int voltage = -1;

        ret = robot.get_tio_vout_param(&enable, &voltage);
        if (ret != ERR_SUCC) {
            std::cerr << "get_tio_vout_param failed for arm " << arm
                      << ", ret=" << ret << std::endl;
        } else {
            std::cout << "Arm " << arm << " TIO power state:" << std::endl;
            std::cout << "  vout_enable = " << enable
                      << "  (0=disabled, 1=enabled)" << std::endl;
            std::cout << "  vout_vol    = " << voltage
                      << "  (0=24V, 1=12V)" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Log out
    robot.login_out();

    return 0;
}
