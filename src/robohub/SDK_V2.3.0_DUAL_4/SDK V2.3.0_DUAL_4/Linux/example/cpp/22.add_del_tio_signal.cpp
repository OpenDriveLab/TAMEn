#include <JAKAZuRobot.h>
#include <iostream>

int main()
{
    JAKAZuRobot robot;
    robot.login_in("192.168.2.200");

    // SignInfo sign_info;
    // sign_info.sig_name = "test_sig";
    // memcpy(sign_info.sig_name, "test_sig", 8);
    // sign_info.chn_id = 1;
    // sign_info.sig_type = 0;
    // sign_info.value = 10;
    // sign_info.frequency = 10;
    // sign_info.sig_addr = 0x1;
    // robot.add_tio_rs_signal(sign_info);

    SignInfo sig_info_arr[8];
    int size = 8;
    int res = robot.get_rs485_signal_info(sig_info_arr, &size);

    std::cout << "res is " << res << "\n";
    
    for (int i = 0; i < size; i++)
    {
        std::cout << sig_info_arr[i].chn_id << "\n";
    }

    robot.login_out();
    return 0;
}