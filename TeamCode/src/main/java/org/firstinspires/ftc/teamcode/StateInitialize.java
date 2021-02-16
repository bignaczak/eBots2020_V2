package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class StateInitialize implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;


    // ***********   CONSTRUCTOR   ***********************
    public StateInitialize(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.INITIALIZE;
        this.nextAutonStateEnum = AutonStateEnum.MOVE_TO_TARGET_ZONE;
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
        return opMode.isStarted();
    }

    @Override
    public void performStateSpecificTransitionActions() {
        //initialize encoders
        AutonParameters autonParameters = robot.getEbotsMotionController().getAutonParameters();
        robot.initializeEncoderTrackers(autonParameters);

    }

    @Override
    public void performStateActions() {
        // There are no stat actions except to
        this.opMode.telemetry.addLine("Stuck in INITIALIZED state, something is wrong");
        this.opMode.telemetry.update();
    }
}
