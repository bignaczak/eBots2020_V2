/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.String.format;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auton_InitSetup", group="Auton")
//@Disabled
public class Auton_InitSetup extends LinearOpMode {

    //initializing and declaring class attributes
    Alliance tempAlliance;
    StartLine.LinePosition startLinePosition;
    StartLine startLine;
    AutonParameters autonParameters;
    Pose.PresetPose startingPose;
    Robot robot;
    TargetZone targetZone;
    LaunchLine launchLine = new LaunchLine();
    AutonState autonState = AutonState.PREMATCH_SETUP;
    StopWatch stateStopWatch = new StopWatch();
    RevBlinkinLedDriver blinkinLedDriver;
    Telemetry.Item patternName;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    boolean isSetupCorrect;
    boolean wasSetupCorrect = false;
    public enum AutonState{
        PREMATCH_SETUP,
        DETECT_STARTER_STACK,
        INITIALIZE,
        MOVE_TO_TARGET_ZONE,
        PLACE_WOBBLE_GOAL,
        MOVE_TO_LAUNCH_LINE,
        SHOOT_POWER_SHOTS,
        PARK_ON_LAUNCH_LINE
    }

    @Override
    public void runOpMode(){

        //initialize time limit
        long stateTimeLimit = 0L;
        tempAlliance = Alliance.RED;
        startLinePosition = StartLine.LinePosition.OUTER;
        startLine = new StartLine(startLinePosition, tempAlliance);
        //todo add constructor that will except startLinePosition
        if (startLinePosition == StartLine.LinePosition.OUTER){
            startingPose = Pose.PresetPose.OUTER_START_LINE;
        } else {
            startingPose = Pose.PresetPose.INNER_START_LINE;
        }
        autonParameters = AutonParameters.DEBUG_THREE_WHEEL;
        robot = new Robot(startingPose, tempAlliance, autonParameters);
        targetZone = new TargetZone(robot.getAlliance(), TargetZone.Zone.B);
        //initialize setup timer
        StopWatch setupStopWatch = new StopWatch();
        //initialize pattern and blinkinLedDriver
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinking");
        //initialize drive wheels
        robot.initializeStandardDriveWheels(hardwareMap);
        //initialize imu
        robot.initializeImu(hardwareMap);
        //initialize color sensors
        robot.initializeColorSensors(hardwareMap);
        //initialize digital touch sensors
        robot.initializeEbotsDigitalTouches(hardwareMap);
        //initialize LED lights
        robot.initializeRevBlinkinLedDriver(hardwareMap);
        //initialize Rev2MeterDistance sensors
        robot.initializeEbotsRev2mDistanceSensors(hardwareMap);
        //prepare expansion hubs for bulk heads
        robot.initializeExpansionHubsForBulkRead(hardwareMap);
        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialize Complete!");
        telemetry.update();



        while (!this.isStarted() & autonState != AutonState.INITIALIZE){
            switch (autonState) {
                case PREMATCH_SETUP:
                    if (isSetupCorrect & setupStopWatch.getElapsedTimeMillis() > 2000 & !this.isStarted()) {
//                      set the new state
                        autonState = AutonState.DETECT_STARTER_STACK;
                        standardStateTransitionActions();
                    } else {
                        //Read the values for the color sensors from hardware into variables
                        for (EbotsColorSensor sensor : robot.getEbotsColorSensors()) {
                            sensor.setColorValue();
                        }

                        //Read the values for the distance sensors from hardware into variables
                        for(EbotsRev2mDistanceSensor distanceSensor: robot.getEbotsRev2mDistanceSensors()){
                            distanceSensor.setDistanceInches();
                        }

                        //Read the values for the digital touch sensors
                        for(EbotsDigitalTouch ebotsDigitalTouch: robot.getEbotsDigitalTouches()){
                            ebotsDigitalTouch.setIsPressed();
                        }

                        RobotSide robotSide;
                        EbotsColorSensor.TapeColor tapeColor;
                        RevBlinkinLedDriver.BlinkinPattern alliancePattern;
                        // Check alliance
                        if (robot.getAlliance() == Alliance.RED) {
                            robotSide = RobotSide.LEFT;
                            tapeColor = EbotsColorSensor.TapeColor.RED;
                            alliancePattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                        } else {
                            robotSide = RobotSide.RIGHT;
                            tapeColor = EbotsColorSensor.TapeColor.BLUE;
                            alliancePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                        }

                        //figure out which side of the robot the tape is on and
                        //calculate distance from the wall the distance sensor to the wall
                        double playFieldWidth = new PlayField().getFieldWidth() /2;
                        double startLineY = startLine.getFieldPosition().getyPosition();
                        double robotWidth = robot.getSizeCoordinate(CsysDirection.Y);
                        double nominalDistance = playFieldWidth - Math.abs(startLineY) - robotWidth;
                        double disTol = 5;
                        double minDistance = nominalDistance + disTol;
                        double maxDistance = nominalDistance - disTol;
                        boolean isOnTape;
                        //isOnWall is equal to the opposite of digital touch
                        EbotsDigitalTouch backWall = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.DETECT_BACK_WALL,
                                robot.getEbotsDigitalTouches());
                        boolean isOnWall = backWall.getIsPressed();
                        boolean isCorrectStartLine = false;

                        RobotSide distanceSide = (robot.getAlliance() == Alliance.RED) ? RobotSide.RIGHT : RobotSide.LEFT;
                        double distance = EbotsRev2mDistanceSensor.getDistanceForRobotSide(distanceSide, robot.getEbotsRev2mDistanceSensors());
                        //check distance for robot side
                        if (distance >= minDistance && distance <= maxDistance){
                            isCorrectStartLine = true;
                        }

                        isOnTape = EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), robotSide, tapeColor);
                        isSetupCorrect = isOnTape && isOnWall && isCorrectStartLine;
                        telemetry.addLine("min and max distance is: " + minDistance + ", " + maxDistance);
                        telemetry.addLine("Robot is on the back wall: " + isOnWall);
                        telemetry.addLine("Robot is on the correct tape: " + isOnTape);
                        telemetry.addLine("Robot is on the correct start line: " + isCorrectStartLine);
                        telemetry.addLine("Overall correct set up: " + isCorrectStartLine);
                        //display LED lights, green is good to go, red means there is a problem in setup
                        pattern = (isSetupCorrect) ? RevBlinkinLedDriver.BlinkinPattern.GREEN : alliancePattern;
                        if (isSetupCorrect & !wasSetupCorrect){
                            setupStopWatch.reset();
                        }
                        wasSetupCorrect = isSetupCorrect;
                        blinkinLedDriver.setPattern(pattern);
                        patternName.setValue(pattern.toString());
                        telemetry.update();
                    }
                    break;
                case DETECT_STARTER_STACK:
                    if (this.isStarted()){
                        //do standard transitional actions
                        autonState = AutonState.INITIALIZE;
                        //set target position
                        TargetZone.Zone observedTarget = StarterStackObservation.getObservedTarget();
                        //todo could be deleted if not needed (line 225)
                        targetZone.setAlliance(robot.getAlliance());
                        targetZone.setZone(observedTarget);
                        Pose targetPose = new Pose(targetZone.getFieldPosition(), 0);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);
                        standardStateTransitionActions();
                    } else {
                        if (tfod != null) {
                            // getUpdatedRecognitions() will return null if no new information is available since
                            // the last time that call was made.
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                telemetry.addData("# Object Detected", updatedRecognitions.size());
                                if (updatedRecognitions.size() == 0 ){
                                     new StarterStackObservation(TargetZone.Zone.A);
                                }
                                // step through the list of recognitions and display boundary info.
                                int i = 0;
                                for (Recognition recognition : updatedRecognitions) {
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                    if (recognition.getHeight() < 40){
                                        new StarterStackObservation(TargetZone.Zone.B);
                                    } else {
                                        //if the height is greater than 40 then the zone is C
                                        new StarterStackObservation(TargetZone.Zone.C);
                                    }
                                }
                                telemetry.update();
                            }
                        }
                    }
                }
            }



        waitForStart();

        telemetry.clearAll();

        while(opModeIsActive()){
            switch (autonState) {
                case INITIALIZE:
                    if (this.isStarted()) {
                        //initialize encoders
                        robot.initializeEncoderTrackers(autonParameters);

                        //set the new state
                        autonState = AutonState.MOVE_TO_TARGET_ZONE;
                        standardStateTransitionActions();

                    } else {
                        telemetry.addLine("ERROR: Stuck in INITIALIZED state, something is wrong");
                        telemetry.update();
                    }
                    break;
                case MOVE_TO_TARGET_ZONE:
                    if (robot.getEbotsMotionController().isTargetPoseReached(robot)
                            | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {
                        //preform transitional actions
                        robot.stop();

                        //set the new state
                        autonState = autonState.PLACE_WOBBLE_GOAL;
                        standardStateTransitionActions();
                        stateTimeLimit = 5000; //set a time limit
                    } else {
                        //preform the state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                        //report telemetry
                        telemetry.addData("Current State ", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                        telemetry.addData("actual pose: ", robot.getActualPose().toString());
                        telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
                        telemetry.addData("Error: ", robot.getPoseError().toString());
                    }
                    break;
                case PLACE_WOBBLE_GOAL:
                    if (stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {
                        //preform transition actions
                        //TBD code to fold Wobble goal arm
                        //Create a new target pose on the launch line in the center of field
                        double xCoord = launchLine.getX() - (robot.getSizeCoordinate(CsysDirection.X) / 2);
                        Pose targetPose = new Pose(xCoord, 0, 0);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //set the new state
                        autonState = AutonState.MOVE_TO_LAUNCH_LINE;
                        standardStateTransitionActions();
                    } else {    //preform the state actions
                        //TBD code to place the wobble goal
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
                    }
                    break;
                case MOVE_TO_LAUNCH_LINE:
                    if (robot.getEbotsMotionController().isTargetPoseReached(robot)
                            | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {
                        //preform transitional actions
                        robot.stop();
                        //TBD spin up the ring launcher

                        //set the new state
                        autonState = AutonState.SHOOT_POWER_SHOTS;
                        standardStateTransitionActions();
                        stateTimeLimit = 5000L;
                    } else {
                        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                        //Report telemetry
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                        telemetry.addData("Actual Pose: ", robot.getActualPose());
                        telemetry.addData("Target Pose: ", robot.getTargetPose());
                        telemetry.addData("Error: ", robot.getPoseError().toString());
                        telemetry.update();
                    }
                    break;
                case SHOOT_POWER_SHOTS:
                    if (stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {
                        robot.stop();

                        //Create a new target pose on the launch line in center of field
                        Pose targetPose = new Pose(launchLine.getX(), 0, 0);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //state the new state
                        autonState = AutonState.PARK_ON_LAUNCH_LINE;
                        standardStateTransitionActions();
                    } else {    //preform state actions
                        //TBD action to launch rings at powerShots
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
                    }
                    break;
                case PARK_ON_LAUNCH_LINE:
                    if (!opModeIsActive()) {  //check for trigger event
                        robot.stop();
                    } else {
                        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                        //Report telemetry
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                        telemetry.addData("Actual Pose: ", robot.getActualPose());
                        telemetry.addData("Target Pose: ", robot.getTargetPose());
                        telemetry.addData("Error: ", robot.getPoseError().toString());
                        telemetry.update();
                    }
                    break;
            }
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void standardStateTransitionActions(){
        stateStopWatch.reset();
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
    }
}
