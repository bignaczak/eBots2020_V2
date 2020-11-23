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
    private int loopCount = 0;
    private PlayField playField = new PlayField();

    StartLine.LinePosition startLinePosition;


    //Debug variables
    private final String logTag = "EBOTS";
    private final boolean debugOn = true;

    @Override
    public void runOpMode() {
        //************************************************************8
        //***   INITIALIZE THE ROBOT
        //************************************************************8
        ElapsedTime myTimer = new ElapsedTime();

        AutonParameters autonParameters = AutonParameters.DEBUG_THREE_WHEEL;
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

        //Initialize the color sensors
        robot.initializeColorSensors(hardwareMap);

        //Initialize the digitalTouch sensors
        robot.initializeEbotsDigitalTouches(hardwareMap);

        //Initialize the LED lights
        robot.initializeRevBlinkinLedDriver(hardwareMap);

        //Initialize the Rev2mDistance Sensors
        robot.initializeEbotsRev2mDistanceSensors(hardwareMap);

        //Prepare the expansion hubs for bulk reads
        robot.initializeExpansionHubsForBulkRead(hardwareMap);

        //Write telemetry
        telemetry.clearAll();
        telemetry.addData("Alliance", robot.getAlliance().toString());
        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
        telemetry.addLine("Initialization complete:  Push X to proceed");
        telemetry.update();

        myTimer.reset();
        double telemetryTimeOutSeconds = 10;        //time to wait in seconds before proceeding
        while(myTimer.seconds() < telemetryTimeOutSeconds
                && !gamepad1.x) {
            //just hold the telemetry
        }

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
        EbotsDigitalTouch startLineDigitalTouch = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.SELECT_START_LINE, robot.getEbotsDigitalTouches());

        ElapsedTime allianceLockoutTimer = new ElapsedTime();
        ElapsedTime startLineLockoutTimer = new ElapsedTime();
        final int lockOutTime = 1000;


        while(opModeIsActive()
                //&& !isStartPositionCorrect
                //&& myTimer.seconds() < timeOutSeconds
                && !(gamepad1.x && gamepad1.y)) {

            //Read input from allianceDigitalTouch
            if (allianceLockoutTimer.milliseconds() >= lockOutTime && allianceDigitalTouch.getIsPressed()){
                toggleAlliance();
                allianceLockoutTimer.reset();
            }

            //Read input from startLineDigitalTouch
            if (startLineLockoutTimer.milliseconds() >= lockOutTime && startLineDigitalTouch.getIsPressed()){
                toggleStartLinePosition();
                startLineLockoutTimer.reset();
            }

            isStartPositionCorrect = isStartPositionCorrect();

            telemetry.addLine("Positioning:  Push X & Y to exit" + myTimer.toString());
            telemetry.addData("Alliance | Start Line:", robot.getAlliance().toString() + " | " + startLinePosition.toString() );
            telemetry.addData("LED Pattern: ", robot.updateLedPattern().toString());
            telemetry.addData("Distance Sensors", EbotsRev2mDistanceSensor.printAll(robot.getEbotsRev2mDistanceSensors()) );
            telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
            telemetry.addData("Is setup correct:", isStartPositionCorrect);
            telemetry.update();
        }

        if(debugOn) Log.d(logTag, "Exiting setup, correct setup detected: " + isStartPositionCorrect);

        myTimer.reset();
        while(opModeIsActive()
                && !gamepad1.x
                && myTimer.seconds() < telemetryTimeOutSeconds) {
            telemetry.addLine("Setup Exited...");
            telemetry.addData("Is setup correct:", isStartPositionCorrect);
            telemetry.addLine("Push X to continue");
            telemetry.update();
        }

        //************************************************************8
        //***   PUT CAMERA THREAD HERE
        //************************************************************8

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.clearAll();

        //************************************************************8
        //***   START OF OPMODE
        //************************************************************8
        StopWatch stopWatch = new StopWatch();
        while(opModeIsActive()) {
            loopCount++;
            for (EbotsColorSensor.TapeColor tc : EbotsColorSensor.TapeColor.values()) {
                for (EbotsColorSensor sensor : robot.getEbotsColorSensors()) {
                    telemetry.addData(sensor.sensorLocation.toString() + " detects" + tc.toString() + ": ", sensor.isColor(tc));
                }
            }

            for (EbotsColorSensor.TapeColor tc : EbotsColorSensor.TapeColor.values()) {
                for (RobotSide rs : RobotSide.values()) {
                    telemetry.addData(rs.toString() + " detects" + tc.toString() + ": ", EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), rs, tc));
                }
            }

            EbotsColorSensor.TapeColor launchLineColor = EbotsColorSensor.TapeColor.WHITE;
            String movementMessage;
            DriveCommand driveCommand = new DriveCommand();
            if (EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), RobotSide.FRONT, launchLineColor)) {
                movementMessage = "None";
                //Don't modify the driveCommand, the default constructor creates 0 magnitude for translate and spin
            } else if (EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_LEFT, robot.getEbotsColorSensors()).isColor(launchLineColor)) {
                movementMessage = "Rotate positive heading angle";
                //don't modify translate components, which are initially set to zero
                //Set the robot to spin by providing a direction (either 1 or -1) and the turnSpeed parameter from autonParameters
                driveCommand.setSpinDrive(1, autonParameters.getSpeed().getTurnSpeed());
            } else if (EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_RIGHT, robot.getEbotsColorSensors()).isColor(launchLineColor)) {
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

            //Log sensor data
            if(debugOn){
                Log.d(logTag, EbotsRev2mDistanceSensor.printAll(robot.getEbotsRev2mDistanceSensors()));
                for(EbotsColorSensor ecs: robot.getEbotsColorSensors()){
                    Log.d(logTag, "Location " + ecs.sensorLocation.toString() + " on white: " + ecs.isWhite());
                }
            }

            telemetry.addData("Timer: ", stopWatch.toString(loopCount));
            telemetry.addData("LED Pattern: ", robot.updateLedPattern().toString());
            telemetry.addData("Distance Sensors", EbotsRev2mDistanceSensor.printAll(robot.getEbotsRev2mDistanceSensors()) );
            telemetry.addData("Movement: ", movementMessage);
            robot.drive();
        }
    }

    private void toggleAlliance() {
        robot.toggleAlliance();
        updateStartPose();
        robot.updateLedPattern();
    }

    private void toggleStartLinePosition() {
        if (startLinePosition == StartLine.LinePosition.INNER) {
            startLinePosition = StartLine.LinePosition.OUTER;
        } else {
            startLinePosition = StartLine.LinePosition.INNER;
        }
        updateStartPose();
    }

    private void updateStartPose(){
        Pose startingPose = calculateStartingPose();     //robot object exists at this point
        robot.setActualPose(startingPose);
    }

    private Pose calculateStartingPose(){
        //Starting poses are handled in an enumeration within Pose

        Pose.PresetPose presetPose;
        if(startLinePosition == StartLine.LinePosition.INNER){
            presetPose = Pose.PresetPose.INNER_START_LINE;
        } else{
            presetPose = Pose.PresetPose.OUTER_START_LINE;
        }

        Pose startingPose = new Pose(presetPose, robot.getAlliance());
        return startingPose;
    }

    private boolean isStartPositionCorrect(){
        EbotsColorSensor.TapeColor startLineColor = EbotsColorSensor.TapeColor.BLUE;
        RobotSide startLineSide = RobotSide.RIGHT;

        if (robot.getAlliance() == Alliance.RED) {
            startLineColor = EbotsColorSensor.TapeColor.RED;
            startLineSide = RobotSide.LEFT;
        }

        //Update whether the start position has been achieved
        return EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), startLineSide, startLineColor);
    }
}
