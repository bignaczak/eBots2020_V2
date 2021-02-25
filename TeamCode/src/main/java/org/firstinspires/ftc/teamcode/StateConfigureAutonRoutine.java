package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class StateConfigureAutonRoutine implements AutonState{
    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;

    EbotsDigitalTouch selectAlliance;
    EbotsDigitalTouch selectStartLine;
    EbotsDigitalTouch selectDelay;
    EbotsRevBlinkinLedDriver ebotsRevBlinkinLedDriver;

    AutonEbotsV1 autonEbotsV1;

    int startDelaySeconds;

    StopWatch lockoutTimer = new StopWatch();
    long lockoutDuration = 750L;

    final boolean debugOn = true;
    final String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateConfigureAutonRoutine(LinearOpMode opModeIn, Robot robotIn) {

        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.CONFIGURE_AUTON_ROUTINE;
        this.nextAutonStateEnum = AutonStateEnum.PREMATCH_SETUP;

        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Instantiating class");

        ArrayList<EbotsDigitalTouch>  digitalTouches = robot.getEbotsDigitalTouches();
        this.selectAlliance = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.SELECT_ALLIANCE, digitalTouches);
        this.selectStartLine = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.SELECT_START_LINE, digitalTouches);
        this.selectDelay = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.SELECT_DELAY, digitalTouches);

        ArrayList<EbotsRevBlinkinLedDriver> ebotsRevBlinkinLedDrivers = robot.getLedDrivers();
        if(debugOn) Log.d(logTag, "Number of ledDrivers: " + ebotsRevBlinkinLedDrivers.size());
        this.ebotsRevBlinkinLedDriver = EbotsRevBlinkinLedDriver.getEbotsRevBlinkinLedDriverByLedLocation(
                EbotsRevBlinkinLedDriver.LedLocation.MAIN, ebotsRevBlinkinLedDrivers);

        autonEbotsV1 = (AutonEbotsV1) opMode;

        opMode.telemetry.clearAll();
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
        // This exits when the user either pushes left bumper and x
        boolean configComplete = (opMode.gamepad1.left_bumper && opMode.gamepad1.x);

        // or pushes the start button
        boolean verdict = configComplete || opMode.isStarted();


        return verdict;
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Reset the staring pose for the robot
        Pose newStartPose = new Pose(autonEbotsV1.getStartLinePosition(), robot.getAlliance());
        robot.setActualPose(newStartPose);
    }

        @Override
    public void performStateActions() {
        // Read the sensor values (this is part of the bulk read)
        for(EbotsDigitalTouch edt: robot.getEbotsDigitalTouches()){
            edt.setIsPressed();
        }

        // Only process signals after lockout timer
        if(lockoutTimer.getElapsedTimeMillis() > lockoutDuration){
            processUserInput();
        }

        ebotsRevBlinkinLedDriver.setAlliancePattern(robot.getAlliance());
        updateTelemetry();
    }


    private void processUserInput() {
        Gamepad gamepad = opMode.gamepad1;

        // Change alliance, startline position, and delay
        if (gamepad.left_bumper && gamepad.x) {
            // Do nothing if both are pressed, that is an exit condition
        } else if (selectAlliance.getIsPressed()) {
            robot.toggleAlliance();
            lockoutTimer.reset();
        } else if (selectStartLine.getIsPressed()) {
            toggleStartLinePosition();
            lockoutTimer.reset();
        } else if (selectDelay.getIsPressed()) {
            if (startDelaySeconds < 20) {
                startDelaySeconds += 5;
            } else{
                startDelaySeconds = 0;
            }
            lockoutTimer.reset();
        }
    }

    private void updateTelemetry(){
        Telemetry t = opMode.telemetry;
        String fmt = "%d";
        opMode.telemetry.addData("Current State: ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine("Push Left Bumper + X on Gamepad1 to proceed");
        t.addData("Alliance: ", robot.getAlliance());
        t.addData("Start Line: ", autonEbotsV1.getStartLinePosition());
        t.addData("Delay: ", String.format(fmt, startDelaySeconds));
        t.update();
    }

    private void toggleStartLinePosition(){
        if(autonEbotsV1.getStartLinePosition() == StartLine.LinePosition.INNER){
            autonEbotsV1.setStartLinePosition(StartLine.LinePosition.OUTER);
        } else{
            autonEbotsV1.setStartLinePosition(StartLine.LinePosition.INNER);
        }
    }

}
