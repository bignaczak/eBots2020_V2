package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class StateInitialize implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;

    boolean debugOn = true;
    String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateInitialize(LinearOpMode opModeIn, Robot robotIn){
        if(debugOn) Log.d(logTag, "Entering StateInitialize::Constructor...");
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.INITIALIZE;
        this.nextAutonStateEnum = AutonStateEnum.MOVE_TO_LAUNCH_LINE;
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
        // Zero all encoders
        robot.zeroEncoders();
    }

    @Override
    public void performStateActions() {
        // There are no stat actions except to
        this.opMode.telemetry.addLine("Stuck in INITIALIZED state, something is wrong");
        this.opMode.telemetry.update();
    }
}
