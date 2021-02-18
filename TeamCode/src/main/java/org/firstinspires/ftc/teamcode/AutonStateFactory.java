package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStateEnum;

public class AutonStateFactory {
    /**
     * This class acts as a factory for AutonStates which implement the AutonState interface
     */

    public AutonState getAutonState(AutonStateEnum autonStateEnum, LinearOpMode opMode, Robot robot){
        AutonState returnState = null;

        if(autonStateEnum == null) {
            returnState = null;
        }else if(autonStateEnum == AutonStateEnum.PREMATCH_SETUP){
            returnState = new StatePrematchSetup(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.DETECT_STARTER_STACK){
            returnState = new StateDetectStarterStack(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.INITIALIZE){
            returnState = new StateInitialize(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_TO_TARGET_ZONE){
            returnState = new StateMoveToTargetZone(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.PLACE_WOBBLE_GOAL){
            returnState = new StatePlaceWobbleGoal(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_TO_LAUNCH_LINE){
            returnState = new StateMoveToLaunchLine(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.SHOOT_POWER_SHOTS){
            returnState = new StateShootPowerShots(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.PARK_ON_LAUNCH_LINE){
            returnState = new StateParkOnLaunchLine(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_FOR_CALIBRATION){
            returnState = new StateMoveForCalibration(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.AWAIT_USER_FEEDBACK){
            returnState = new StateAwaitUserFeedback(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.SET_PID_COEFFICIENTS){
            returnState = new StateSetPidCoefficients(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.CONFIGURE_AUTON_ROUTINE){
            returnState = new StateConfigureAutonRoutine(opMode, robot);
        }
        return returnState;
    }


}
