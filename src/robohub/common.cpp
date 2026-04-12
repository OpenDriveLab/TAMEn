#include "common.h"
#include "JAKAZuRobot.h"

// Check and clear faults. Some faults require a power cycle and cannot be cleared directly.
int check_error(JAKAZuRobot &robot, bool need_reset)
{
    int error[2] = {};
    robot.robot_is_in_error(error);
    if (error[0] || error[1])
    {
        printf("error happens\n");
        if (need_reset)
        {
            robot.clear_error();    // Clear faults before enabling to avoid enable failures
        }
        return 1;
    }
    return 0;
}

// Query power and servo-enable state
int check_state(JAKAZuRobot &robot)
{
    RobotState state;
    robot.get_robot_state(&state);
    printf("Dual Robot State: %s %s %s\n", state.estoped ? "estoped" : "Safe", state.poweredOn ? "powered" : "off", state.servoEnabled ? "enabled" : "disabled");
    return 0;
}

int check_inpos(JAKAZuRobot &robot)
{
    int inpos[2] = {};
    robot.robot_is_inpos(inpos);
    if (inpos[0] && inpos[1])
    {
        printf("Both robots are inpos\n");
        return 1;
    }
    printf("inpos state %d %d\n", inpos[0], inpos[1]);
    return 0;
}