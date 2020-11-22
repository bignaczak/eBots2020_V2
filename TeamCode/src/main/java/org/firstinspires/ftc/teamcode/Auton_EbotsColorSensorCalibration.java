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

@Autonomous(name="EbotsColorSensorCal", group="Concept")
//@Disabled
public class Auton_EbotsColorSensorCalibration extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private StopWatch stopWatch;
    private Robot robot;
    private int loopCount = 0;
    private PlayField playField = new PlayField();

    //Debug variables
    private final String logTag = "EBOTS";
    private final boolean debugOn = true;
    boolean firstPassInitLoop = true;
    boolean firstPassLoop = true;

    @Override
    public void runOpMode() {
        //************************************************************8
        //***   INITIALIZE THE ROBOT
        //************************************************************8
        //Setup for the Robot
        Alliance alliance = Alliance.BLUE;
        double allianceSign = (alliance == Alliance.BLUE) ? 1 : -1;  //used to flip signs of dimensions if red
        StartLine.LinePosition startLinePosition = StartLine.LinePosition.Inner;

        AutonParameters autonParameters = AutonParameters.DEBUG_THREE_WHEEL;
        if(debugOn) Log.d(logTag, "autonParameters created! " + autonParameters.toString());

        //Setup the starting pose
        //Start on the bottom wall at the inner blue start line, heading = 0
        //Note, because this is used to create a post prior to robot instantiation, must access RobotSize enum directly
        double xCenter = -playField.getFieldHeight()/2 + Robot.RobotSize.xSize.getSizeValue()/2;
        //To find y dimension, must create a start line
        StartLine startLine = new StartLine(startLinePosition, alliance);
        //The yCenter assumes right wheels are on line if blue ane left wheels if red
        double yCenter = startLine.getSizeCoordinate(CsysDirection.Y) + (Robot.RobotSize.ySize.getSizeValue()/2 * allianceSign);
        Pose startingPose = new Pose(xCenter,yCenter, 90);
        if(debugOn) Log.d(logTag, "startingPose created! " + startingPose.toString());

        robot = new Robot(startingPose, alliance, autonParameters);
        if(debugOn) Log.d(logTag, "startingPose assigned to Robot!");

        //Set the target pose
        Pose targetPose = new Pose(0,0,0);
        robot.setTargetPose(targetPose);
        if(debugOn) Log.d(logTag, "targetPose created! " + robot.getTargetPose().toString());

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

        //Prepare the expansion hubs for bulk reads
        robot.initializeExpansionHubsForBulkRead(hardwareMap);

        //Write telemetry
        telemetry.clearAll();
        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
        telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //************************************************************8
        //***   END OF INITIALIZATION
        //************************************************************8


        //************************************************************8
        //***   VERIFY STARTING POSITION OF ROBOT
        //************************************************************8
        boolean startPositionVerified = false;
        ElapsedTime myTimer = new ElapsedTime();
        double timeOutSeconds = 15;

        while(opModeIsActive()
                && !startPositionVerified
                && myTimer.seconds() < timeOutSeconds
                && !(gamepad1.x | gamepad1.y | gamepad1.a | gamepad1.b)) {
            EbotsColorSensor.TapeColor startLineColor = EbotsColorSensor.TapeColor.BLUE;
            EbotsColorSensor.RobotSide startLineSide = EbotsColorSensor.RobotSide.RIGHT_SIDE;
            if (alliance == Alliance.RED) {
                startLineColor = EbotsColorSensor.TapeColor.RED;
                startLineSide = EbotsColorSensor.RobotSide.LEFT_SIDE;
            }

            if (EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), startLineSide, startLineColor)) {
                startPositionVerified = true;
            }

            telemetry.addData("Timer:", myTimer.toString());
            telemetry.addData("Is setup correct:", startPositionVerified);
            telemetry.update();
        }

        myTimer.reset();
        while(myTimer.seconds() < 5) {
            telemetry.addLine("Setup Exited...");
            telemetry.addData("Is setup correct:", startPositionVerified);
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

        while(opModeIsActive()) {
            for (EbotsColorSensor.TapeColor tc : EbotsColorSensor.TapeColor.values()) {
                for (EbotsColorSensor sensor : robot.getEbotsColorSensors()) {
                    telemetry.addData(sensor.sensorLocation.toString() + " detects" + tc.toString() + ": ", sensor.isColor(tc));
                }
            }

            for (EbotsColorSensor.TapeColor tc : EbotsColorSensor.TapeColor.values()) {
                for (EbotsColorSensor.RobotSide rs : EbotsColorSensor.RobotSide.values()) {
                    telemetry.addData(rs.toString() + " detects" + tc.toString() + ": ", EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), rs, tc));
                }
            }

            EbotsColorSensor.TapeColor launchLineColor = EbotsColorSensor.TapeColor.WHITE;
            String movementMessage;
            DriveCommand driveCommand = new DriveCommand();
            if (EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), EbotsColorSensor.RobotSide.FRONT_SIDE, launchLineColor)) {
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
            telemetry.addData("Movement: ", movementMessage);
            robot.drive();
        }

    }
}
