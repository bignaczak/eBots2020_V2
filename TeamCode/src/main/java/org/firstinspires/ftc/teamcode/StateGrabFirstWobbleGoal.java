package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class StateGrabFirstWobbleGoal implements AutonState{

    //Position the robot and grab the first wobble goal

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    DcMotorEx crane;
    int cranePos = 0;
    long timeout = 3000L;
    StopWatch stateTimer;

    boolean debugOn = true;
    String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateGrabFirstWobbleGoal(LinearOpMode opModeIn, Robot robotIn){
        if(debugOn) Log.d(logTag, "Entering StateInitialize::Constructor...");
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.GRAB_FIRST_WOBBLE_GOAL;
        this.nextAutonStateEnum = AutonStateEnum.MOVE_TO_TARGET_ZONE;
        stateTimer = new StopWatch();
        crane = robot.getCrane();
    }

    // ***********   GETTERS    ***********************
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
        // This condition should be immediately satisfied

        return opMode.opModeIsActive();
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Grab the wobble goal

    }

    @Override
    public void performStateActions() {
        // move if needed

        String f = "%.2f";
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addData("Crane  Power", String.format(f,crane.getPower()));
        opMode.telemetry.addData("crane Position", cranePos);
        opMode.telemetry.update();

    }
}
