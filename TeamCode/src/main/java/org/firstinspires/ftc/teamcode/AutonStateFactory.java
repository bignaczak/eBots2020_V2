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
        }else if(autonStateEnum == AutonStateEnum.UNFOLD_CRANE){
            returnState = new StateUnfoldCrane(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.GRAB_FIRST_WOBBLE_GOAL){
            returnState = new StateGrabFirstWobbleGoal(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_TO_TARGET_ZONE){
            returnState = new StateMoveToTargetZone(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.PLACE_WOBBLE_GOAL){
            returnState = new StatePlaceWobbleGoal(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_TO_LAUNCH_LINE){
            returnState = new StateMoveToLaunchLine(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.SHOOT_POWER_SHOTS){
            returnState = new StateShootPowerShots(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_TO_SECOND_START_LINE){
            returnState = new StateMoveToSecondStartLine(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.PICKUP_SECOND_WOBBLE_GOAL){
            returnState = new StatePickupSecondWobbleGoal(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.MOVE_TO_TARGET_ZONE_AGAIN){
            returnState = new StateMoveToTargetZoneAgain(opMode, robot);
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
        }else if(autonStateEnum == AutonStateEnum.SPIN_360_DEGREES){
            returnState = new StateSpin360Degrees(opMode, robot);
        }else if(autonStateEnum == AutonStateEnum.TEST_CONTROL_LOOP_SPEED){
            returnState = new StateTestControlLoopSpeed(opMode, robot);
        }
        return returnState;
    }


}
