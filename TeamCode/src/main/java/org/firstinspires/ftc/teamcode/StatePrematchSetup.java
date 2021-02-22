package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;

public class StatePrematchSetup implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;


    boolean isSetupCorrect;
    boolean wasSetupCorrect = false;
    StopWatch setupStopWatch = new StopWatch();
    long previousLoopEnd;
    RobotSide robotSide;
    EbotsColorSensor.TapeColor tapeColor;
    EbotsRevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern patternPositionVerified = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    double nominalDistance;
    double distanceTolerance = 10;
    double actualDistance;

    TelemetryScreen telemetryScreen = TelemetryScreen.A;
    StopWatch lockoutTimer = new StopWatch();
    long buttonLockoutLimit = 1000L;

    final boolean debugOn = true;
    final String logTag = "EBOTS";

    private enum TelemetryScreen{
        A,B
    }

    // ***********   CONSTRUCTOR   ***********************
    public StatePrematchSetup(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.PREMATCH_SETUP;
        this.nextAutonStateEnum = AutonStateEnum.DETECT_STARTER_STACK;
        ledDriver = EbotsRevBlinkinLedDriver.getEbotsRevBlinkinLedDriverByLedLocation(EbotsRevBlinkinLedDriver.LedLocation.MAIN, robot.getLedDrivers());

    }


    // ***********   GETTERS    ***********************
    @Override
    public AutonStateEnum getNextAutonStateEnum() {
        return this.nextAutonStateEnum;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return this.currentAutonStateEnum;
    }


    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        long stableSetupTimeTarget = 5000L;
        boolean isCorrectSetupStable = setupStopWatch.getElapsedTimeMillis() > stableSetupTimeTarget;

        // Verify that the setup is stable:
        //  a) that it is correct AND
        //  b) has been correct for some period of time (stableSetupTimeTarget)
        boolean isSetupStable = (isSetupCorrect & isCorrectSetupStable);

        boolean manualOverride = (opMode.gamepad1.left_bumper && opMode.gamepad1.x && setupStopWatch.getElapsedTimeMillis()>1500) ? true : false;
        // Exit if either:
        //  a) setup is stable OR
        //  b) opMode is started
        //  c) manual override
        boolean verdict = (isSetupStable | this.opMode.isStarted() | manualOverride);
        return verdict;
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Perform a light show to verify exiting
        StopWatch blinkTimer = new StopWatch();
        long blinkTimeLimit = 750L;
        int numBlinks = 3;

        for(int i=0; i<numBlinks; i++){
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            while(!opMode.isStarted() && !opMode.isStopRequested() && blinkTimer.getElapsedTimeMillis()<blinkTimeLimit){
                //just wait
            }
            ledDriver.setPattern(patternPositionVerified);
            while(!opMode.isStarted() && !opMode.isStopRequested() && blinkTimer.getElapsedTimeMillis()<blinkTimeLimit){
                //just wait
            }
        }

        // Set Alliance color
        ledDriver.setAlliancePattern(robot.getAlliance());
    }

    @Override
    public void performStateActions() {
        //if(debugOn) Log.d(logTag, currentAutonStateEnum + ": entering performStateActions");
        // Update the readings from the sensors
        performSensorHardwareReads();
        // Determine robotSide, tapeColor, and alliancePattern
        assignMeasurementParameters();

        // Now check if three conditions are met
        //  1) Touching the back wall
        //  2) Positioned over the startline tape
        //  3) On the correct startline
        boolean isOnWall = isTouchingBackWall();
        boolean isOnTape = isCorrectRobotSideOnCorrectColorTape();
        boolean isCorrectStartLine = isRobotPlacedOnCorrectStartLine();

        // Provide the verdict for setup correctness for this pass through loop
        isSetupCorrect = isOnTape && isOnWall && isCorrectStartLine;

        // If the setup just turned from incorrect to correct, reset the setupStopWatch
        // Exit conditions require the setup to be stable for some period of time
        // which is measured with the setupStopWawtch
        if (isSetupCorrect & !wasSetupCorrect){
            setupStopWatch.reset();
        }

        // Use this to pass to the next loop iteration
        wasSetupCorrect = isSetupCorrect;

        // switch telemetry screens when press bumpers
        if((lockoutTimer.getElapsedTimeMillis() > buttonLockoutLimit)
                && (opMode.gamepad1.left_bumper | opMode.gamepad1.right_bumper)){
            shiftTelemetryScreen();
            lockoutTimer.reset();
        }

        updateTelemetry();

        updateLedDisplay();

        previousLoopEnd = setupStopWatch.getElapsedTimeMillis();

    }

    // ***********   CLASS MEMBER METHODS   ***********************
    private void updateLedDisplay() {
        //display LED lights, green is good to go, red means there is a problem in setup
        if(isSetupCorrect){
            ledDriver.setPattern(patternPositionVerified);
        } else{
            ledDriver.setAlliancePattern(robot.getAlliance());
        }
    }

    private boolean isTouchingBackWall() {
        EbotsDigitalTouch backWall = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.DETECT_BACK_WALL,
                robot.getEbotsDigitalTouches());
        return backWall.getIsPressed();
    }

    private boolean isCorrectRobotSideOnCorrectColorTape() {
        return EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), robotSide, tapeColor);
    }

    private boolean isRobotPlacedOnCorrectStartLine() {
        boolean isCorrectStartLine = false;

        //calculate expected distance from the wall the distance sensor to the wall
        double playFieldHalfWidth = new PlayField().getFieldWidth() /2;

        // this is refactored to use the robot's currentPose, which was set based on the startLine
        // double nominalDistance = playFieldHalfWidth - Math.abs(startLineY) - robotWidth;
        nominalDistance = playFieldHalfWidth - Math.abs(robot.getActualPose().getY()) -
                (robot.getSizeCoordinate(CsysDirection.Y) / 2);

        // set which distance sensor will provide the reading
        RobotSide distanceSide = (robot.getAlliance() == Alliance.RED) ? RobotSide.RIGHT : RobotSide.LEFT;
        actualDistance = EbotsRev2mDistanceSensor.getDistanceForRobotSide(distanceSide, robot.getEbotsRev2mDistanceSensors());

        //check distance for robot side
        if (actualDistance >= (nominalDistance - distanceTolerance)
                && actualDistance <= (nominalDistance + distanceTolerance)){
            isCorrectStartLine = true;
        }

        return isCorrectStartLine;
    }


    private void assignMeasurementParameters() {
        // Check alliance
        if (robot.getAlliance() == Alliance.RED) {
            robotSide = RobotSide.LEFT;
            tapeColor = EbotsColorSensor.TapeColor.RED;
        } else {
            robotSide = RobotSide.RIGHT;
            tapeColor = EbotsColorSensor.TapeColor.BLUE;
        }
    }


    private void performSensorHardwareReads() {
        long loopDuration = setupStopWatch.getElapsedTimeMillis() - previousLoopEnd;
        robot.bulkReadSensorInputs(loopDuration,true,true);
        robot.updateAllSensorValues();

//        //Read the values for the color sensors from hardware into variables
//        for (EbotsColorSensor sensor : robot.getEbotsColorSensors()) {
//            sensor.setColorValue();
//        }
//
//        //Read the values for the distance sensors from hardware into variables
//        for(EbotsRev2mDistanceSensor distanceSensor: robot.getEbotsRev2mDistanceSensors()){
//            distanceSensor.setDistanceInches();
//        }
//
//        //Read the values for the digital touch sensors
//        for(EbotsDigitalTouch ebotsDigitalTouch: robot.getEbotsDigitalTouches()){
//            ebotsDigitalTouch.setIsPressed();
//        }
    }

    private void shiftTelemetryScreen(){
        if(telemetryScreen == TelemetryScreen.A){
            telemetryScreen = TelemetryScreen.B;
        } else{
            telemetryScreen = TelemetryScreen.A;
        }
        opMode.telemetry.clearAll();
    }


    private void updateTelemetry(){
        if(telemetryScreen == TelemetryScreen.A) {
            this.opMode.telemetry.addLine("Current autonState: " + this.currentAutonStateEnum.toString());
            this.opMode.telemetry.addLine("opMode is Started / Active: " + opMode.isStarted() + "/" + opMode.opModeIsActive());
            this.opMode.telemetry.addLine("Setup Config: " + robot.getAlliance() + " | " + robot.getActualPose().toString());
            this.opMode.telemetry.addLine("Actual -- Expected[<-->]: " +
                    String.format("%.2f", (this.actualDistance)) + " -- " +
                    String.format("%.2f", (nominalDistance - distanceTolerance)) + "<-->" +
                    ", " + String.format("%.2f", (nominalDistance + distanceTolerance)));
            this.opMode.telemetry.addLine("Robot is on the back wall: " + isTouchingBackWall());
            this.opMode.telemetry.addLine("Robot is on the correct tape: " + isCorrectRobotSideOnCorrectColorTape());
            this.opMode.telemetry.addLine("Robot is on the correct start line: " + isRobotPlacedOnCorrectStartLine());
            this.opMode.telemetry.addLine("Overall correct set up: " + isSetupCorrect + " - " + setupStopWatch.toString());
            ArrayList<EbotsRevBlinkinLedDriver> ledDrivers = robot.getLedDrivers();
            EbotsRevBlinkinLedDriver ledDriver = EbotsRevBlinkinLedDriver.getEbotsRevBlinkinLedDriverByLedLocation(EbotsRevBlinkinLedDriver.LedLocation.MAIN, ledDrivers);
            this.opMode.telemetry.addLine("LED pattern: " + ledDriver.getLedLocation());
        } else {
            // Read out the encoders and report the values in telemetry
            opMode.telemetry.addData("Heading", robot.getActualPose().getHeadingDeg());
            for(EncoderTracker e: robot.getEncoderTrackers()){
                opMode.telemetry.addLine(e.toString());
            }
        }
        this.opMode.telemetry.update();

    }
}
