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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="Auton_ShaanBatra", group="Auton")
//@Disabled
public class Auton_ShaanBatra extends LinearOpMode {
    AutonParameters autonParameters = AutonParameters.DEBUG_THREE_WHEEL;
    Robot robot = new Robot(Pose.PresetPose.OUTER_START_LINE, Alliance.BLUE, autonParameters);
    TargetZone targetZone = new TargetZone(robot.getAlliance(), TargetZone.Zone.B);
    LaunchLine launchLine = new LaunchLine();

    AutonState autonState = AutonState.INITIALIZE;
    StopWatch stateStopWatch = new StopWatch();
    public enum AutonState{
        INITIALIZE,
        MOVE_TO_TARGET_ZONE,
        PLACE_WOBBLE_GOAL,
        MOVE_TO_LAUNCH_LINE,
        SHOOT_POWER_SHOTS,
        PARK_ON_LAUNCH_LINE,
    }


    @Override
    public void runOpMode() {

        robot.initializeStandardDriveWheels(hardwareMap);

        robot.initializeImu(hardwareMap);

        robot.initializeColorSensors(hardwareMap);

        robot.initializeEbotsDigitalTouches(hardwareMap);

        robot.initializeRevBlinkinLedDriver(hardwareMap);

        robot.initializeEbotsRev2mDistanceSensors(hardwareMap);

        robot.initializeExpansionHubsForBulkRead(hardwareMap);

        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialized Complete!");
        telemetry.update();


        waitForStart();
        telemetry.clearAll();
        long stateTimeLimit = 0L;

    while (opModeIsActive()){
        switch (autonState){
            case INITIALIZE:
                if(this.isStarted()) {
                    robot.initializeEncoderTrackers(autonParameters);

                    Pose targetPose = new Pose(targetZone.getFieldPosition(), 0);
                    robot.setTargetPose(targetPose);
                    stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                    autonState = AutonState.MOVE_TO_TARGET_ZONE;
                    standardStateTransitionActions();
                }else {
                    telemetry.addLine("Stuck in INITIALIZED state, something is wrong");
                    telemetry.update();
                }
                break;

            case MOVE_TO_TARGET_ZONE:
                if(robot.getEbotsMotionController().isTargetPoseReached(robot)
                |stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){
                robot.stop();

                autonState = AutonState.PLACE_WOBBLE_GOAL;
                standardStateTransitionActions();
                stateTimeLimit = 5000;
            } else {
                robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                telemetry.addData("Current State", autonState.toString());
                telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                telemetry.addData("Actual Pose:", robot.getActualPose().toString());
                telemetry.addData("Target Pose:", robot.getTargetPose().toString());
                telemetry.addData("Error: ", robot.getPoseError().toString());
                telemetry.update();

            }
            break;
            case PLACE_WOBBLE_GOAL:
                if(stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {
                    double xCoord = launchLine.getX()-(robot.getSizeCoordinate(CsysDirection.X)/2);
                    Pose targetPose = new Pose (xCoord , 0, 0);
                    robot.setTargetPose(targetPose);
                    stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);

                    autonState = AutonState.MOVE_TO_LAUNCH_LINE;
                    standardStateTransitionActions();
                } else{
                    telemetry.addData("Current State", autonState.toString());
                    telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
                }
                break;
            case MOVE_TO_LAUNCH_LINE:
                if(robot.getEbotsMotionController().isTargetPoseReached(robot)
                    |stateStopWatch.getElapsedTimeMillis() > stateTimeLimit) {
                    robot.stop();
                    autonState = AutonState.SHOOT_POWER_SHOTS;
                    standardStateTransitionActions();
                    stateTimeLimit = 5000L;

            } else {
                    robot  .getEbotsMotionController().moveToTargetPose(robot,stateStopWatch);

                    telemetry.addData("Current State", autonState.toString());
                    telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                    telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
                    telemetry.addData("Target Pose:  ", robot.getTargetPose().toString());
                    telemetry.addData("Error: ", robot.getPoseError().toString());
                    telemetry.update();
            }
                break;
            case SHOOT_POWER_SHOTS:
                if (stateStopWatch.getElapsedTimeMillis() > stateTimeLimit){
                    robot.stop();
                    Pose targetPose = new Pose(launchLine.getX(), 0, 0);
                    robot.setTargetPose(targetPose);
                    stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);
                    autonState = AutonState.PARK_ON_LAUNCH_LINE;
                    standardStateTransitionActions();

                } else {
                    telemetry.addData("Current State", autonState.toString());
                    telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);

                }
                break;
            case PARK_ON_LAUNCH_LINE:
                if (!opModeIsActive()){
                    robot.stop();
                } else {
                    robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
                    telemetry.addData("Current State", autonState.toString());
                    telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
                    telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
                    telemetry.addData("Target Pose:", robot.getTargetPose().toString());
                    telemetry.addData("Error:", robot.getPoseError().toString());
                    telemetry.update();
        }
                break;
        }
    }
    }
    public void standardStateTransitionActions(){
        stateStopWatch.reset();
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
    }
}

