package org.firstinspires.ftc.teamcode;

public enum AutonStateEnum {
    CONFIGURE_AUTON_ROUTINE,
    PREMATCH_SETUP,
    DETECT_STARTER_STACK,
    INITIALIZE,
    MOVE_TO_TARGET_ZONE,
    PLACE_WOBBLE_GOAL,
    MOVE_TO_LAUNCH_LINE,
    SHOOT_POWER_SHOTS,
    PARK_ON_LAUNCH_LINE,
    COMPLETED,

    //********  THE FOLLOWING STATES ARE UTILITY STATES *********,
    SET_PID_COEFFICIENTS,
    MOVE_FOR_CALIBRATION,
    AWAIT_USER_FEEDBACK
}
