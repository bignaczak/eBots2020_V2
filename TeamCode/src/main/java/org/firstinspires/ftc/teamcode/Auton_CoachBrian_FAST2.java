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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Auton_CoachBrianFAST2", group="Auton")
//@Disabled
public class Auton_CoachBrian_FAST2 extends LinearOpMode {
    //Declare and initialize class attributes
    AutonParameters autonParameters;
    Robot robot;
    TargetZone targetZone;
    LaunchLine launchLine;

    AutonState autonState;
    StopWatch stateStopWatch;

    //Debug variables
    private final String logTag = "EBOTS";
    private final boolean debugOn = true;

    public enum AutonState{
        INITIALIZE,
        MOVE_TO_TARGET_ZONE,
        PLACE_WOBBLE_GOAL,
        MOVE_TO_LAUNCH_LINE,
        SHOOT_POWER_SHOTS,
        PARK_ON_LAUNCH_LINE
    }
    @Override
    public void runOpMode(){
        //When the robot begins, it is in initialized state
        //In this state, initialize the robot sensors and actuators
        //and wait for the event: driver pushes START
        //************************************************************
        //***   INITIALIZE THE ROBOT
        //************************************************************
        autonParameters = AutonParameters.DEBUG_TWO_WHEEL;
        autonParameters.setSpeed(Speed.FAST);
        autonParameters.getSpeed().setK_p(0.1);
        autonParameters.getSpeed().setK_i(0.2);
        robot = new Robot(Pose.PresetPose.INNER_START_LINE, Alliance.RED, autonParameters);

        targetZone = new TargetZone(robot.getAlliance(),TargetZone.Zone.B);
        launchLine = new LaunchLine();

        autonState = AutonState.INITIALIZE;
        stateStopWatch = new StopWatch();

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
                        //Perform the transitional actions
                        //Initialize the encoders
                        robot.initializeEncoderTrackers(autonParameters);

                        //Set the target pose (where the robot should drive to + heading)
                        Pose targetPose = new Pose(targetZone.getFieldPosition(), 0);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //Set the new state
                        autonState = AutonState.MOVE_TO_TARGET_ZONE;
                        standardStateTransitionActions();

                    } else {
                        telemetry.addLine("Stuck in INITIALIZED state, something is wrong");
                        telemetry.update();
                    }
                    break;

                case MOVE_TO_TARGET_ZONE:
                    if (robot.getEbotsMotionController().isTargetPoseReached(robot)      //check if trigger event occurred
                            | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {

                        //Perform transitional actions
                        robot.stop();

                        //Set the new state
                        autonState = AutonState.PLACE_WOBBLE_GOAL;
                        standardStateTransitionActions();
                        stateTimeLimit = 5000;     //set a timelimit of 5 seconds for next state
                    } else {
                        //Perform the state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                        //Report telemetry
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
                        telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
                        telemetry.addData("Error: ", robot.getPoseError().toString());
                        telemetry.update();
                    }
                    break;

                case PLACE_WOBBLE_GOAL:
                    if(stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){  //trigger event?
                        //Perform transitional actions
                        //TBD code to fold Wobble Arm
                        //Create a new target pose on the launch line in center of field
                        double xCoord = launchLine.getX()-(robot.getSizeCoordinate(CsysDirection.X)/2);
                        Pose targetPose = new Pose(xCoord, robot.getActualPose().getY(), 0);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //Set the new state
                        autonState = AutonState.MOVE_TO_LAUNCH_LINE;
                        standardStateTransitionActions();
                    } else {   //perform the state actions
                        //TBD code to place the wobble goal
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
                    }
                    break;

                case MOVE_TO_LAUNCH_LINE:
                    if(robot.getEbotsMotionController().isTargetPoseReached(robot)          //trigger event?
                            | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){      //or timed out?
                        //Perform transitional actions
                        robot.stop();
                        //TBD spin up the ring launcher

                        //Set the new state
                        autonState = AutonState.SHOOT_POWER_SHOTS;
                        standardStateTransitionActions();
                        stateTimeLimit = 5000L;
                    } else {                    //perform the state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot,stateStopWatch);
                        //Report telemetry
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
                        telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
                        telemetry.addData("Error: ", robot.getPoseError().toString());
                        telemetry.update();
                    }
                    break;
                case SHOOT_POWER_SHOTS:
                    if(stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){     //trigger event?
                        robot.stop();

                        //Create a new target pose on the launch line in center of field
                        Pose targetPose = new Pose(launchLine.getX(), robot.getActualPose().getY(), 180);
                        robot.setTargetPose(targetPose);
                        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                        //Set the new state
                        autonState = AutonState.PARK_ON_LAUNCH_LINE;
                        standardStateTransitionActions();

                    } else {            //perform state actions
                        //TBD action to launch rings at powershots
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
                    }
                    break;
                case PARK_ON_LAUNCH_LINE:
                    if(!opModeIsActive()){      //check for trigger event
                        robot.stop();
                    } else {                //perform state actions
                        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                        //Report telemetry
                        telemetry.addData("Current State", autonState.toString());
                        telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                        telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
                        telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
                        telemetry.addData("Error: ", robot.getPoseError().toString());
                        telemetry.update();
                    }
                    break;
            }
        }

        //Exit opmode
        robot.stop();

        //State debug info
//        if (debugOn) {
//            Log.d(logTag, "Transitioning out of state " + autonState.toString());
//            if (robot.getEbotsMotionController().isTargetPoseReached(robot)) {
//                Log.d(logTag, "Pose Achieved in " + format("%.2f", stateStopWatch.getElapsedTimeSeconds()));
//            } else {
//                Log.d(logTag, "Failed to reach target, timed out!!! " + robot.getPoseError().toString());
//            }
//        }

    }

    public void standardStateTransitionActions(){
        stateStopWatch.reset();
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
    }

}
