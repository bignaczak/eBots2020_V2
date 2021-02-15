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

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.AutonStateEnum.INITIALIZE;


/**
 * This is an autonomous routine using a state machine
 * It is derived off linear opmode and traverses states which implement
 * AutonState interface
 */

@Autonomous(name="AutonEbotsV1", group="Auton")
//@Disabled
public class AutonEbotsV1 extends LinearOpMode {

    //initializing and declaring class attributes
    private AutonParameters autonParameters = AutonParameters.DEBUG_TWO_WHEEL;
    private Robot robot;
    private LaunchLine launchLine = new LaunchLine();
    private StopWatch stateStopWatch = new StopWatch();

    private AutonStateFactory autonStateFactory = new AutonStateFactory();
    private AutonState autonState;
    private AutonStateEnum targetAutonStateEnum = AutonStateEnum.PREMATCH_SETUP;


    @Override
    public void runOpMode(){
        long stateTimeLimit = 0L;
        StartLine.LinePosition startLinePosition = StartLine.LinePosition.OUTER;
        Alliance tempAlliance = Alliance.BLUE;
        Pose startingPose = new Pose(startLinePosition, tempAlliance);

        // Adjust the auton parameters before instantiating robot
        autonParameters.setSpeed(Speed.FAST);
        robot = new Robot(startingPose, tempAlliance, autonParameters);
        initializeRobot();

        autonState = autonStateFactory.getAutonState(targetAutonStateEnum, this, robot);

        while (!this.isStarted() & autonState.getCurrentAutonStateEnum() != INITIALIZE){
            switch (autonState.getCurrentAutonStateEnum()) {
                case PREMATCH_SETUP:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;

                case DETECT_STARTER_STACK:
                    if (autonState.areExitConditionsMet()){
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;
            }
        }

        waitForStart();

        telemetry.clearAll();

        while(opModeIsActive()){
            switch (targetAutonStateEnum) {
                case INITIALIZE:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;
                case MOVE_TO_TARGET_ZONE:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;
                case PLACE_WOBBLE_GOAL:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {    //preform the state actions
                        autonState.performStateActions();
                    }
                    break;
                case MOVE_TO_LAUNCH_LINE:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;
                case SHOOT_POWER_SHOTS:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;
                case PARK_ON_LAUNCH_LINE:
                    if (autonState.areExitConditionsMet()) {
                        // Perform state-specific transition actions
                        autonState.performStateSpecificTransitionActions();
                        // Perform standard transition actions, including setting the next autonState
                        performStandardStateTransitionActions();
                    } else {
                        autonState.performStateActions();
                    }
                    break;
            }
        }
    }

    private void initializeRobot() {
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
    }

    public void performStandardStateTransitionActions(){
        stateStopWatch.reset();
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
        //Set the next AutonState
        autonState = autonStateFactory.getAutonState(autonState.getNextAutonStateEnum(), this, robot);
    }

}
