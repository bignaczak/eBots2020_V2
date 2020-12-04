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

@Autonomous(name="EbotsCalLinear", group="Concept")
//@Disabled
public class Auton_EbotsCalibrationLinear extends LinearOpMode {

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
        AutonParameters autonParameters = AutonParameters.SIMULATED_TWO_WHEEL;
        if(debugOn) Log.d(logTag, "autonParameters created! " + autonParameters.toString());

        //Start on the bottom wall
        Pose startingPose = new Pose(-playField.getFieldHeight()/2,0, 90);
        //Pose startingPose = new Pose(-playField.getFieldHeight(),0, 90);
        //Pose startingPose = new Pose(0,0, 90);
        if(debugOn) Log.d(logTag, "startingPose created!");
        robot = new Robot(startingPose, Alliance.BLUE, autonParameters);
        if(debugOn) Log.d(logTag, "startingPose assigned to Robot!");

        //Set the target pose
        Pose targetPose = new Pose(0,0,0);
        robot.setTargetPose(targetPose);
        if(debugOn) Log.d(logTag, "targetPose created!");

        //initialize the wheels and encoders
        robot.initializeStandardDriveWheels(hardwareMap);
        if(debugOn) Log.d(logTag, "Initialized driveWheels!");
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.clearAll();


        robot.getEbotsMotionController().moveToTargetPose(robot, new StopWatch());

        //targetPose = new Pose(62, -62, 180);
        //robot.setTargetPose(targetPose);
        //robot.getEbotsMotionController().moveToTargetPose(robot, (LinearOpMode) this);
    }
}