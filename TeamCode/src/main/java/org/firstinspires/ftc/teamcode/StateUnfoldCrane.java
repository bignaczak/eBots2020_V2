package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class StateUnfoldCrane implements AutonState{

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
    public StateUnfoldCrane(LinearOpMode opModeIn, Robot robotIn){
        if(debugOn) Log.d(logTag, "Entering StateInitialize::Constructor...");
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.UNFOLD_CRANE;
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
        boolean shouldExit = (cranePos >= 300) | !opMode.opModeIsActive()
                | (stateTimer.getElapsedTimeMillis() > timeout);
        return shouldExit;
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Zero all encoders
        robot.zeroEncoders();
    }

    @Override
    public void performStateActions() {
        // unfold the crane
        cranePos = crane.getCurrentPosition();
        if (cranePos < 65) {
            crane.setPower(1);
        } else if (cranePos < 240){
            crane.setPower(0.5);
        } else {
            crane.setPower(0.);
        }

        String f = "%.2f";
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addData("Crane  Power", String.format(f,crane.getPower()));
        opMode.telemetry.addData("crane Position", cranePos);
        opMode.telemetry.update();

    }
}
