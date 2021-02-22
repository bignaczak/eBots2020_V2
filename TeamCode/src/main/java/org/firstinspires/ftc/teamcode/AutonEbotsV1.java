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
import android.view.animation.AnimationUtils;

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
    private AutonParameters autonParameters;
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

        while (opModeIsActive() | !isStarted()){
            if(autonState.getCurrentAutonStateEnum()== INITIALIZE){
                // Must include wait for start for opMode to run
                waitForStart();
            }
            if (autonState.areExitConditionsMet()) {
                // Perform state-specific transition actions
                autonState.performStateSpecificTransitionActions();
                // Perform standard transition actions, including setting the next autonState
                performStandardStateTransitionActions();
            } else {
                autonState.performStateActions();
            }
        }


    }


    public void performStandardStateTransitionActions(){
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
        //Set the next AutonState
        autonState = autonStateFactory.getAutonState(autonState.getNextAutonStateEnum(), this, robot);
    }


    private void initializeRobot() {
        if(debugOn) Log.d(logTag, "Entering AutonEbotsV1::initializeRobot...");
        Alliance tempAlliance = Alliance.BLUE;
        Pose startingPose = new Pose(startLinePosition, tempAlliance);

        // Adjust the auton parameters before instantiating robot
//        autonParameters = AutonParameters.CALIBRATION_TWO_WHEEL;
        autonParameters = AutonParameters.DEBUG_TWO_WHEEL;
        autonParameters.setSpeed(Speed.FAST);
        // Encoder setup must be re-written since enum is singleton
        autonParameters.setEncoderSetup(EncoderSetup.THREE_WHEELS);

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
        // preapare encoderTrackers
        robot.initializeEncoderTrackers(autonParameters);
        //  Note CALIBRATION_TWO_WHEEL requires the Encoder Setup be changed after initialization
        if(autonParameters == AutonParameters.CALIBRATION_TWO_WHEEL){
            // During robot creation, the encoder setup was set to THREE_WHEELS to instantiate all three encoders
            // This must be switched back to TWO_WHEELS after instantiation so navigation used TWO_WHEEL algorithm
            // Specifically, this affects PoseChange::calculateRobotMovement() and PoseChange::calculateSpinAngle()
            // Note: enum is singleton, so this must be reset next time this routine is run
            autonParameters.setEncoderSetup(EncoderSetup.TWO_WHEELS);
        }

        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialize Complete!");
        telemetry.update();
    }



}
