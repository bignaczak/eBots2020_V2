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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.String.format;

/**
 * This opmode is designed to provide calibration information for the encoders
 * Goal:
 * --Calibrate the distance traveled
 * --Calibrate the 3-Wheel Auton heading calculations
 */
@Autonomous(name="Auton_EncoderCalibration", group="Auton")
@Disabled
public class Auton_EncoderCalibration extends LinearOpMode {
    //Declare and initialize class attributes
    AutonParameters autonParameters = AutonParameters.DEBUG_THREE_WHEEL;
    Pose startPose = new Pose (0, new PlayField().getYCoordTouchingWall(0),0);
    Robot robot = new Robot(startPose, Alliance.RED, autonParameters);
    TargetZone targetZone = new TargetZone(robot.getAlliance(),TargetZone.Zone.B);
    LaunchLine launchLine = new LaunchLine();

    AutonState autonState = AutonState.INITIALIZE;
    StopWatch stateStopWatch = new StopWatch();

    //Debug variables
    private final String logTag = "EBOTS";
    private final boolean debugOn = true;

    public enum AutonState{
        INITIALIZE,
        MOVE_TO_FIELD_CENTER,
        WAIT_FOR_DISTANCE_VERIFICATION,
        SPIN_360_POSITIVE,
        SPIN_360_NEGATIVE,
        DEFEND_POSITION
    }
    @Override
    public void runOpMode(){
        //When the robot begins, it is in initialized state
        //In this state, initialize the robot sensors and actuators
        //and wait for the event: driver pushes START
        //************************************************************
        //***   INITIALIZE THE ROBOT
        //************************************************************
        //Initialize the wheels
        robot.initializeStandardDriveWheels(hardwareMap);

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

        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialization Complete!");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        long stateTimeLimit = 0L;

        while(opModeIsActive()){
            switch (autonState) {
                case INITIALIZE:
                    if (this.isStarted()) {       //EVENT TO TRIGGER NEXT STATE IS PUSHING START BUTTON
                        logExitingState();
                        //Perform the transitional actions
                        //Initialize the encoders
                        robot.initializeEncoderTrackers(autonParameters);

                        //Set the target pose (where the robot should drive to + heading)
                        Pose targetPose = new Pose(0,0, 0);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //Set the new state
                        autonState = AutonState.MOVE_TO_FIELD_CENTER;
                        standardStateTransitionActions();

                    } else {
                        telemetry.addLine("Stuck in INITIALIZED state, something is wrong");
                        telemetry.update();
                    }
                    break;

                case MOVE_TO_FIELD_CENTER:
                    if (robot.getEbotsMotionController().isTargetPoseReached(robot)      //check if trigger event occurred
                            | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {

                        logExitingState();
                        //Perform transitional actions
                        robot.stop();

                        //Set the new state
                        autonState = AutonState.WAIT_FOR_DISTANCE_VERIFICATION;
                        standardStateTransitionActions();
                        //no timer for the next state
                    } else {
                        //Perform the state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                        //Report telemetry
                        displayMovementTelemetry();

                    }
                    break;

                case WAIT_FOR_DISTANCE_VERIFICATION:
                    if (!gamepad1.right_bumper && gamepad1.x) {

                        logExitingState();
                        //Perform transitional actions
                        robot.stop();
                        Pose targetPose = new Pose(robot.getActualPose().getFieldPosition(), 120);
                        robot.setTargetPose(targetPose);

                        //Set the new state
                        autonState = AutonState.SPIN_360_POSITIVE;
                        standardStateTransitionActions();
                        stateTimeLimit = 10000;     //set a timelimit of 5 seconds for next state
                    } else {
                        //Perform the state actions
                        telemetry.addLine("Push Right Bumper and X to continue...");

                        //Report telemetry
                        displayMovementTelemetry();

                    }
                    break;

                case SPIN_360_POSITIVE:
                    if(stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){  //trigger event?
                        //Perform transitional actions
                        logExitingState();
                        //Create a new target pose on the launch line in center of field
                        Pose targetPose = new Pose(robot.getActualPose().getFieldPosition(), -120);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //Set the new state
                        autonState = AutonState.SPIN_360_NEGATIVE;
                        standardStateTransitionActions();

                    } else {   //perform the state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot,stateStopWatch);

                        //look at the target pose and keep moving it clockwise until it is back to 0
                        if(Math.abs(robot.getPoseError().getHeadingErrorDeg()) < 15     //if getting close to target
                                && robot.getTargetPose().getHeadingDeg() != 0){         //and target isn't back to zero already
                            //bump the target by an additional 120 deg
                            double newHeadingTarget = Pose.applyAngleBound(robot.getTargetPose().getHeadingDeg() + 120);
                            robot.getTargetPose().setHeadingDeg(newHeadingTarget);
                        }

                        //Update telemetry
                        displayMovementTelemetry();

                    }
                    break;

                case SPIN_360_NEGATIVE:
                    if(robot.getEbotsMotionController().isTargetPoseReached(robot)          //trigger event?
                            | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){      //or timed out?
                        //Perform transitional actions
                        logExitingState();
                        robot.stop();
                        //TBD spin up the ring launcher

                        //Set the new state
                        autonState = AutonState.DEFEND_POSITION;
                        standardStateTransitionActions();
                        //no time limit for next state
                    } else {                    //perform the state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot,stateStopWatch);

                        //look at the target pose and keep moving it clockwise until it is back to 0
                        if(Math.abs(robot.getPoseError().getHeadingErrorDeg()) < 15     //if getting close to target
                                && robot.getTargetPose().getHeadingDeg() != 0){         //and target isn't back to zero already
                            //bump the target by an additional 120 deg
                            double newHeadingTarget = Pose.applyAngleBound(robot.getTargetPose().getHeadingDeg() - 120);
                            robot.getTargetPose().setHeadingDeg(newHeadingTarget);
                        }

                        //Report telemetry
                        displayMovementTelemetry();

                    }
                    break;

                case DEFEND_POSITION:
                    if(!opModeIsActive()){      //or timed out?
                        //Perform transitional actions
                        logExitingState();
                        robot.stop();

                        //Set the new state
                        //No next state
                    } else {                    //perform the state actions
                        // Make sure the current position is maintained
                        robot.getEbotsMotionController().moveToTargetPose(robot,stateStopWatch);
                        //Report telemetry
                        displayMovementTelemetry();
                    }
                    break;

            }
        }

        //Exit opmode
        robot.stop();

        //State debug info


    }

    public void standardStateTransitionActions(){
        stateStopWatch.reset();
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
        logStateTransition();
    }

    public void logStateTransition(){
        if(debugOn){
            Log.d(logTag, "Transitioning into state " + autonState.toString());
            Log.d(logTag, robot.toString());
        }
    }

    public void logExitingState(){
        if (debugOn) {
            Log.d(logTag, "Transitioning out of state " + autonState.toString() + " after " + stateStopWatch.toString());
            if (robot.getEbotsMotionController().isTargetPoseReached(robot)) {
                Log.d(logTag, "Pose Achieved in " + format("%.2f", stateStopWatch.getElapsedTimeSeconds()));
            } else {
                Log.d(logTag, "Failed to reach target, timed out!!! " + robot.getPoseError().toString());
            }
        }
    }

    public void displayMovementTelemetry(){
        telemetry.addData("Current State", autonState.toString());
        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
        telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
        telemetry.addData("Error: ", robot.getPoseError().toString());
        telemetry.update();
    }

}
