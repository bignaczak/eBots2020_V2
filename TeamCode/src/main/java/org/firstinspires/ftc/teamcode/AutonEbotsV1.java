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

    private AutonStateFactory autonStateFactory = new AutonStateFactory();
    private AutonState autonState;
    private StartLine.LinePosition startLinePosition = StartLine.LinePosition.OUTER;


    final boolean debugOn = true;
    final String logTag = "EBOTS";

    public StartLine.LinePosition getStartLinePosition() {
        return startLinePosition;
    }

    public void setStartLinePosition(StartLine.LinePosition startLinePosition) {
        this.startLinePosition = startLinePosition;
    }

    @Override
    public void runOpMode(){
        if(debugOn) Log.d(logTag, "Entering runOpMode for AutonEbotsV1");
        initializeRobot();

        autonState = autonStateFactory.getAutonState(AutonStateEnum.CONFIGURE_AUTON_ROUTINE, this, robot);

        while (!this.isStarted() & autonState.getCurrentAutonStateEnum() != INITIALIZE){
            if(debugOn) Log.d(logTag, "Entering state machine before wait for start");
            switch (autonState.getCurrentAutonStateEnum()) {
                case CONFIGURE_AUTON_ROUTINE:
                case PREMATCH_SETUP:
                case DETECT_STARTER_STACK:
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
        // todo: This can be eliminated and all states can follow the same structure
        waitForStart();

        while(opModeIsActive()){
            switch (autonState.getCurrentAutonStateEnum()) {
                case INITIALIZE:
                case MOVE_TO_TARGET_ZONE:
                case PLACE_WOBBLE_GOAL:
                    //preform the state actions
                case MOVE_TO_LAUNCH_LINE:
                case SHOOT_POWER_SHOTS:
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

        Alliance tempAlliance = Alliance.BLUE;
        Pose startingPose = new Pose(startLinePosition, tempAlliance);

        // Adjust the auton parameters before instantiating robot
        autonParameters.setSpeed(Speed.FAST);
        robot = new Robot(startingPose, tempAlliance, autonParameters);


        //initialize drive wheels
        robot.initializeStandardDriveWheels(hardwareMap);
        //initialize imu
        robot.initializeImu(hardwareMap);
        //initialize color sensors
        robot.initializeColorSensors(hardwareMap);
        //initialize digital touch sensors
        robot.initializeEbotsDigitalTouches(hardwareMap);
        //initialize LED lights
        robot.initializeEbotsRevBlinkinDriver(hardwareMap);
        //initialize Rev2MeterDistance sensors
        robot.initializeEbotsRev2mDistanceSensors(hardwareMap);
        //prepare expansion hubs for bulk heads
        robot.initializeExpansionHubsForBulkRead(hardwareMap);

        // Note:  encoderTrackers are not loaded until the Initialize state

        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialize Complete!");
        telemetry.update();
    }

    public void performStandardStateTransitionActions(){
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
        //Set the next AutonState
        autonState = autonStateFactory.getAutonState(autonState.getNextAutonStateEnum(), this, robot);
    }



}
