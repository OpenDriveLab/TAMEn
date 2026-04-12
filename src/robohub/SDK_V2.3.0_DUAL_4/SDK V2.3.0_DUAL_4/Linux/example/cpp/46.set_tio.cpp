#include <JAKAZuRobot.h>
#include <chrono>
#include <thread>
#include <iostream>

int main()
{
    JAKAZuRobot robot;
    errno_t ret;
    int N = 5; // Number of cycles

    // Log in to the robot
    ret = robot.login_in("192.168.2.200");
    if (ret != ERR_SUCC) { std::cerr << "Login failed\n"; return -1; }

    // Power on
    ret = robot.power_on();
    if (ret != ERR_SUCC) { std::cerr << "Power on failed\n"; return -1; }

    // Clear errors
    ret = robot.clear_error();
    if (ret != ERR_SUCC) { std::cerr << "Clear error failed\n"; }

    // --- Configure TIO power ---
    ret = robot.set_tio_vout_param(0, 1, 0); // Left arm 24V
    ret = robot.set_tio_vout_param(1, 1, 0); // Right arm 24V
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // --- Verify TIO voltage settings ---
    int vout_enable, vout_vol;
    robot.get_tio_vout_param(0, &vout_enable, &vout_vol);
    std::cout << "Left-arm TIO voltage output: " << vout_enable << ", voltage: " << (vout_vol==0?"24V":"12V") << "\n";
    robot.get_tio_vout_param(1, &vout_enable, &vout_vol);
    std::cout << "Right-arm TIO voltage output: " << vout_enable << ", voltage: " << (vout_vol==0?"24V":"12V") << "\n";

    // --- Configure TIO pin mode ---
    ret = robot.set_tio_pin_mode(0, 1, 0x1); // DO pin mode
    ret = robot.set_tio_pin_mode(1, 1, 0x1);

    // --- Verify TIO pin mode ---
    int pin_mode;
    robot.get_tio_pin_mode(0, 1, &pin_mode);
    std::cout << "Left-arm DO pin mode: 0x" << std::hex << pin_mode << "\n";
    robot.get_tio_pin_mode(1, 1, &pin_mode);
    std::cout << "Right-arm DO pin mode: 0x" << std::hex << pin_mode << "\n";

    // --- Configure RS485 ---
    ModRtuComm comm = {1,7, 115200, 8 ,0 ,0};
    robot.set_rs485_chn_comm(0, comm);
    robot.set_rs485_chn_comm(1, comm);
    robot.set_rs485_chn_mode(0, 1, 1); // Raw RS485
    robot.set_rs485_chn_mode(1, 1, 1);

    // --- Verify RS485 configuration ---
    ModRtuComm getComm;
    int chn_mode;
    robot.get_rs485_chn_comm(0, 1, &getComm);
    robot.get_rs485_chn_mode(0, 1, &chn_mode);
    std::cout << "Left-arm RS485 channel 1 mode: " << chn_mode << ", slave ID: " << getComm.slaveId << "\n";

    // Right arm
    robot.get_rs485_chn_comm(1, 1, &getComm);
    robot.get_rs485_chn_mode(1, 1, &chn_mode);
    std::cout << "Right-arm RS485 channel 1 mode: " << chn_mode << ", slave ID: " << getComm.slaveId << "\n";

    // --- Initialize grippers ---
    uint8_t initCmd[8] = {0x1, 0x6, 0x1, 0x0, 0x0, 0x1, 0x49, 0xf6};
    robot.send_tio_rs_command(0, 1, initCmd, 8);
    robot.send_tio_rs_command(1, 1, initCmd, 8);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // --- Open/close loop ---
    uint8_t openCmd[8]  = {0x1, 0x6, 0x1, 0x3, 0x01, 0xF4, 0x78, 0x21}; // Position 500
    uint8_t closeCmd[8] = {0x1, 0x6, 0x1, 0x3, 0x03, 0xE8, 0x78, 0x88}; // Position 1000

    for (int i = 0; i < N; ++i)
    {
        std::cout << "Cycle " << i+1 << ": retract gripper\n";
        robot.send_tio_rs_command(0, 1, openCmd, 8);
        robot.send_tio_rs_command(1, 1, openCmd, 8);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Cycle " << i+1 << ": open gripper\n";
        robot.send_tio_rs_command(0, 1, closeCmd, 8);
        robot.send_tio_rs_command(1, 1, closeCmd, 8);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // --- Disable TIO power ---
    robot.set_tio_vout_param(0, 0, 0);
    robot.set_tio_vout_param(1, 0, 0);

    // Log out
    robot.login_out();

    return 0;
}
