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

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.AutonStateEnum.INITIALIZE;


/**
 * This is an autonomous routine using a state machine
 * It is derived off linear opmode and traverses states which implement
 * AutonState interface
 */

@Autonomous(name="AutonEbotsV1_Calibration", group="Auton")
//@Disabled
public class AutonEbotsV1_Calibration extends LinearOpMode {

    //initializing and declaring class attributes
    private AutonParameters autonParameters = AutonParameters.DEBUG_TWO_WHEEL;
    private Robot robot;
    private ArrayList<Pose> poseArray= new ArrayList<>();

    private AutonStateFactory autonStateFactory = new AutonStateFactory();
    private AutonState autonState;
    private boolean firstPass = true;

    final boolean debugOn = true;
    final String logTag = "EBOTS";

    @Override
    public void runOpMode(){
        if(debugOn) Log.d(logTag, "Entering runOpMode for AutonEbotsV1");
        initializeRobot();
        initializePoseArray();
        autonState = autonStateFactory.getAutonState(AutonStateEnum.SET_PID_COEFFICIENTS, this, robot);

        while(opModeIsActive() | !isStarted()){
            // todo:  add StateSpin360 from Auton_EncoderCalibration
            switch (autonState.getCurrentAutonStateEnum()) {
                case SET_PID_COEFFICIENTS:

                case MOVE_FOR_CALIBRATION:

                case AWAIT_USER_FEEDBACK:
                case SPIN_360_DEGREES:
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
        waitForStart();
    }

    private void initializeRobot() {
        if(debugOn) Log.d(logTag, "Entering AutonEbotsV1Calibration::initializeRobot...");
        Alliance tempAlliance = Alliance.BLUE;

        // Start against back wall in middle of field
        // Note, since the robot hasn't been instantiated yet, the size is grabbed directly from the enum
        double startX = (new PlayField().getFieldHeight()-Robot.RobotSize.xSize.getSizeValue())/2 * -1;
        Pose startingPose = new Pose(startX,0,0);
        telemetry.clearAll();
        telemetry.addLine("Start robot against back wall on X axis (Y=0)");

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
        robot.initializeRevBlinkinLedDriver(hardwareMap);
        //initialize Rev2MeterDistance sensors
        robot.initializeEbotsRev2mDistanceSensors(hardwareMap);
        //prepare expansion hubs for bulk heads
        robot.initializeExpansionHubsForBulkRead(hardwareMap);

        // Note:  encoderTrackers are not loaded until the Initialize state
        robot.initializeEncoderTrackers(autonParameters);
        if(debugOn) Log.d(logTag, "AutonEbotsV1Calibration::initializeRobot Actual: " + robot.getActualPose().toString());

        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialize Complete!");
        telemetry.update();
    }

    private void performStandardStateTransitionActions(){
        robot.getEbotsMotionController().resetLoopVariables();
        //Set the next AutonState
        autonState = autonStateFactory.getAutonState(autonState.getNextAutonStateEnum(), this, robot);
        firstPass = true;
    }

    private void initializePoseArray(){
        // Move robot forward to center of field
        poseArray.add(new Pose(0,0,0));

        // Move robot back 60 inches to center of field
        poseArray.add(new Pose(-60,0,0));

        // Move robot left
        poseArray.add(new Pose(-60,48,0));

        // Move robot right
        poseArray.add(new Pose(-60,0,0));

        // Move diagonally forward and left
        poseArray.add(new Pose(-12,48,0));

        // Move diagonally back and right
        poseArray.add(new Pose(-60,0,0));

    }

    public Pose getNextPose(){
        Pose returnPose;
        try {
            returnPose = poseArray.remove(0);
        } catch (IndexOutOfBoundsException e){
            returnPose = null;
        }
        return returnPose;
    }

}
