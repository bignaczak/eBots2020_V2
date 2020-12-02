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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This opmode combines all the sensors that the team has been working on
 * into a single autonomous routine
 */

@Autonomous(name="EbotsAllSensorCal", group="Concept")
//@Disabled
public class Auton_EbotsAllSensorCalibration extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    private PlayField playField = new PlayField();

    StartLine.LinePosition startLinePosition = StartLine.LinePosition.INNER;


    //Debug variables
    private final String logTag = "EBOTS";
    private final boolean debugOn = true;

    @Override
    public void runOpMode() {
        //************************************************************8
        //***   INITIALIZE THE ROBOT
        //************************************************************8
        ElapsedTime myTimer = new ElapsedTime();

        AutonParameters autonParameters = AutonParameters.DEBUG_TWO_WHEEL;
        if(debugOn) Log.d(logTag, "autonParameters created! " + autonParameters.toString());

        //Instantiate robot, defaults to Alliance.Blue StartLinePosition.Inner
        robot = new Robot(autonParameters);
        if(debugOn) Log.d(logTag, "startingPose assigned to Robot!");

        //Initialize the wheels
        robot.initializeStandardDriveWheels(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized driveWheels!");

        //Initialize the encoders
        robot.initializeEncoderTrackers(autonParameters);
        if(debugOn) Log.d(logTag, "Initialized encoders!");

        //Initialize the imu
        robot.initializeImu(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized imu!");

        //Initialize the color sensors
        robot.initializeColorSensors(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized color sensors!");

        //Initialize the digitalTouch sensors
        robot.initializeEbotsDigitalTouches(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized digital touch sensors!");

        //Initialize the LED lights
        robot.initializeRevBlinkinLedDriver(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized blinkin led driver!");

        //Initialize the Rev2mDistance Sensors
        robot.initializeEbotsRev2mDistanceSensors(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized distance sensors!");

        //Prepare the expansion hubs for bulk reads
        robot.initializeExpansionHubsForBulkRead(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized expansion hubs for bulk read!");

        //Write telemetry
        telemetry.clearAll();
        telemetry.addData("Alliance", robot.getAlliance().toString());
        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
        telemetry.addLine("Push left bumper + a to test drivewheel motors");
        telemetry.addLine("Initialization complete:  Push X to proceed");
        telemetry.addData("Opmode Status:",  opModeIsActive());
        telemetry.update();

        myTimer.reset();
        double telemetryTimeOutSeconds = 10;        //time to wait in seconds before proceeding

        while(myTimer.seconds() < telemetryTimeOutSeconds
                && !gamepad1.x
                && !isStopRequested()
                && !isStarted()
                ) {
            //just hold the telemetry
            //Note:  opMode is not active unti after the start button is pushed
            if(gamepad1.left_bumper && gamepad1.a){
                robot.testMotors((LinearOpMode) this, gamepad1);
                myTimer.reset();
            }
        }
        if(debugOn) Log.d(logTag, "Initialization complete...");

        //************************************************************8
        //***   END OF INITIALIZATION
        //************************************************************8


        //************************************************************8
        //***   SET ALLIANCE & STARTING POSITION & VERIFY STARTING POSITION
        //************************************************************8
        boolean isStartPositionCorrect = false;
        double timeOutSeconds = 15;
        myTimer.reset();

        //Assign digitalTouch buttons for setting opmode alliance and starting position
        EbotsDigitalTouch allianceDigitalTouch = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.SELECT_ALLIANCE, robot.getEbotsDigitalTouches());
        //EbotsDigitalTouch startLineDigitalTouch = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.SELECT_START_LINE, robot.getEbotsDigitalTouches());

        ElapsedTime allianceLockoutTimer = new ElapsedTime();
        ElapsedTime startLineLockoutTimer = new ElapsedTime();
        final int lockOutTime = 1000;

        StopWatch stopWatch = new StopWatch();
        long loopStartMillis;
        long loopEndMillis = 0L;
        long loopDuration = 0L;
        int loopCount = 0;

        if(debugOn) Log.d(logTag, "Entering start position alignment loop...");

        while(!(gamepad1.x && gamepad1.y)
                && !this.isStopRequested()
                && !this.isStarted()
                //&& !isStartPositionCorrect
                //&& myTimer.seconds() < timeOutSeconds
                //&& opModeIsActive()   //opmode not active until after start button pushed
                ) {
            //if(debugOn) Log.d(logTag, "Top of start position alignment loop");
            loopStartMillis = loopEndMillis;
            loopCount++;

            //Perform bulk read of sensors
            robot.bulkReadSensorInputs(loopDuration);   //reading puts the values in temp storage
            robot.updateAllSensorValues();              //update the values after getting the readings
            //if(debugOn) Log.d(logTag, "Sensor data read...");

            boolean rightTrigger = (gamepad1.right_trigger>0.3);
            //Read input from allianceDigitalTouch
            if (allianceLockoutTimer.milliseconds() >= lockOutTime && !rightTrigger && allianceDigitalTouch.getIsPressed()){
                toggleAlliance();
                allianceLockoutTimer.reset();
            }

            //Read input from startLineDigitalTouch (must push gamepad right trigger & digitalTouch)
            if (startLineLockoutTimer.milliseconds() >= lockOutTime && rightTrigger && allianceDigitalTouch.getIsPressed()){
                toggleStartLinePosition();
                startLineLockoutTimer.reset();
            }


            //todo:  get rid of the boolean variable
            isStartPositionCorrect = robot.isStartPositionCorrect();

            telemetry.addLine("Positioning:  Push X & Y to exit " + myTimer.toString());
            telemetry.addData("Opmode/StopRequest Status:",  this.opModeIsActive() + " / " + this.isStopRequested());
            telemetry.addData("Alliance | Start Line:", robot.getAlliance().toString() + " | " + startLinePosition.toString() );
            telemetry.addData("LED Pattern: ", robot.updateLedPattern().toString());
            telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
            telemetry.addData("Dist.", EbotsRev2mDistanceSensor.printAll(robot.getEbotsRev2mDistanceSensors()) );
            telemetry.addLine(EbotsColorSensor.printColorsObserved(robot.getEbotsColorSensors()));
            telemetry.addData("FL Color: ", EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_LEFT, robot.getEbotsColorSensors()).toString());
            telemetry.addData("Encoders:", EncoderTracker.printAll(robot.getEncoderTrackers()));
            telemetry.addData("Is setup correct:", isStartPositionCorrect);
            telemetry.update();

            loopEndMillis = stopWatch.getElapsedTimeMillis();
            loopDuration = loopEndMillis - loopStartMillis;

            if(debugOn) {
                robot.logSensorData(logTag);
                stopWatch.toString(loopCount, loopDuration);
            }

        }

        if(debugOn) {
            Log.d(logTag, "isStopRequested: " + this.isStopRequested());
            Log.d(logTag, "Exiting setup, correct setup detected: " + isStartPositionCorrect);
        }

        myTimer.reset();
        while(!isStopRequested()
                && !isStarted()
                && !gamepad1.x
                && myTimer.seconds() < telemetryTimeOutSeconds
                && !isStopRequested()
                ) {
            telemetry.addLine("Setup Exited...");
            telemetry.addData("Is setup correct:", isStartPositionCorrect);
            telemetry.addLine("Push X to continue");
            telemetry.update();
        }

        //************************************************************8
        //***   PUT CAMERA THREAD HERE
        //************************************************************8

        // Wait for the game to start (driver presses PLAY)
        if (!this.isStarted()) waitForStart();
        runtime.reset();
        telemetry.clearAll();

        //************************************************************8
        //***   START OF OPMODE
        //************************************************************8
        if(debugOn) Log.d(logTag, "Entering main opmode loop...");
        loopCount = 0;
        stopWatch.startTimer();     //Reset timer
        loopEndMillis = 0L;
        loopDuration = 0L;
        long splitTimeMillis;
        EbotsColorSensor.TapeColor launchLineColor = EbotsColorSensor.TapeColor.WHITE;
        String movementMessage;

        telemetry.clearAll();
        telemetry.update();

        ElapsedTime toggleMotorsTimer = new ElapsedTime();
        boolean engageDriveMotors = false;

        while(!isStopRequested()) {
            if(debugOn) Log.d(logTag, "Top of main control loop...");
            loopCount++;
            loopStartMillis = loopEndMillis;

            if(debugOn) splitTimeMillis = stopWatch.getElapsedTimeMillis();
            robot.bulkReadSensorInputs(loopDuration, loopCount, stopWatch);
            if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, "bulkReadSensorInputs", splitTimeMillis, loopCount);

            robot.updateAllSensorValues();
            if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, "updateAllSensorValues", splitTimeMillis, loopCount);

            DriveCommand driveCommand = new DriveCommand();
            if (EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), RobotSide.FRONT, launchLineColor)) {
                movementMessage = "None";
                //Don't modify the driveCommand, the default constructor creates 0 magnitude for translate and spin
            } else if (robot.getActualPose().getHeadingDeg() > 3){
                movementMessage = "Rotate right, rotated too far";
                driveCommand.setSpinDrive(-1, autonParameters.getSpeed().getTurnSpeed());
            } else if (robot.getActualPose().getHeadingDeg() < -3){
                movementMessage = "Rotate left, rotated too far";
                driveCommand.setSpinDrive(1, autonParameters.getSpeed().getTurnSpeed());
            }else if (EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_LEFT, robot.getEbotsColorSensors()).getObservedColor() == launchLineColor) {
                movementMessage = "Rotate positive heading angle";
                //don't modify translate components, which are initially set to zero
                //Set the robot to spin by providing a direction (either 1 or -1) and the turnSpeed parameter from autonParameters
                driveCommand.setSpinDrive(1, autonParameters.getSpeed().getTurnSpeed());
            } else if (EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_RIGHT, robot.getEbotsColorSensors()).getObservedColor() == launchLineColor) {
                movementMessage = "Rotate negative heading angle";
                //don't modify translate components, which are initially set to zero
                //Set the robot to spin by providing a direction (either 1 or -1) and the turnSpeed parameter from autonParameters
                driveCommand.setSpinDrive(-1, autonParameters.getSpeed().getTurnSpeed());
            } else {
                movementMessage = "Forward";
                //Update the magnitude based on speed parameters from autonParameters
                driveCommand.setMagnitude(autonParameters.getSpeed().getMaxSpeed());
                //don't modify angle or spin component, which are initially set to zero
            }
            if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, "driveCommandLogic", splitTimeMillis, loopCount);

            //Assign the drive command to the robot and calculate drive powers
            robot.setDriveCommand(driveCommand);
            if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, "setDriveCommand", splitTimeMillis, loopCount);

            //Toggle whether drive motors are on
            if(gamepad1.left_bumper && gamepad1.a && toggleMotorsTimer.milliseconds() > lockOutTime){
                engageDriveMotors = !engageDriveMotors;
                toggleMotorsTimer.reset();
            }

            //Drive if you wanna
            if(engageDriveMotors) {
                robot.drive();
            } else {
                robot.stop();
            }
            if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, "robot.drive()", splitTimeMillis, loopCount);


            telemetry.addLine("Push Left Bumper + a to toggle engage motors - " + engageDriveMotors);
            telemetry.addLine(robot.getDriveCommand().toString());
            telemetry.addLine("Actual " + robot.getActualPose().toString());
            telemetry.addData("Dist. ", EbotsRev2mDistanceSensor.printAll(robot.getEbotsRev2mDistanceSensors()) );
            telemetry.addData("Color ", EbotsColorSensor.printColorsObserved(robot.getEbotsColorSensors()));
            telemetry.addData("Movement", movementMessage);
            telemetry.addData("Encoders", EncoderTracker.printAll(robot.getEncoderTrackers()));

            telemetry.update();

            loopEndMillis = stopWatch.getElapsedTimeMillis();
            loopDuration = loopEndMillis - loopStartMillis;
            //Log sensor data
            if(debugOn) {
                robot.logSensorData(logTag);
                Log.d(logTag, stopWatch.toString(loopCount, loopDuration));
            }

        }
        robot.stop();
    }

    private void toggleAlliance() {
        robot.toggleAlliance();
        robot.updateStartPose(startLinePosition);
        robot.updateLedPattern();
    }

    private void toggleStartLinePosition() {
        if (startLinePosition == StartLine.LinePosition.INNER) {
            startLinePosition = StartLine.LinePosition.OUTER;
        } else {
            startLinePosition = StartLine.LinePosition.INNER;
        }
        robot.updateStartPose(startLinePosition);
    }


}
