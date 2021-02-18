package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class StateSetPidCoefficients implements AutonState{
    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;


    final boolean debugOn = false;
    final String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateSetPidCoefficients(LinearOpMode opModeIn, Robot robotIn) {
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Instantiating class");

        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.AWAIT_USER_FEEDBACK;
        this.nextAutonStateEnum = AutonStateEnum.MOVE_FOR_CALIBRATION;

        opMode.telemetry.addLine("Push Left Bumper + X on Gamepad1 to proceed");
        opMode.telemetry.update();

    }


    // ***********   GETTERS   ***********************
    @Override
    public AutonStateEnum getNextAutonStateEnum() {
        return nextAutonStateEnum;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return currentAutonStateEnum;
    }



    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // No exit condition in this method because relies on
        return (opMode.gamepad1.left_bumper && opMode.gamepad1.x);
    }

    @Override
    public void performStateSpecificTransitionActions() {
    }

        @Override
    public void performStateActions() {
    }

}
