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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Concept: EBotsCalibration", group = "Concept")
//@Disabled
public class Auton_EbotsCalibration extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private StopWatch stopWatch;
  private org.firstinspires.ftc.teamcode.Robot robot;
  private int loopCount = 0;
  private PlayField playField = new PlayField();

  //Debug variables
  private final String logTag = "EBOTS";
  private final boolean debugOn = true;
  boolean firstPassInitLoop = true;
  boolean firstPassLoop = true;

  @Override
  public void init() {
    if(debugOn) Log.d(logTag, "Entering init...");
    AutonParameters autonParameters = AutonParameters.SIMULATED_TWO_WHEEL;
    if(debugOn) Log.d(logTag, "autonParameters created!");

    //Start on the bottom wall
    Pose startingPose = new Pose(-playField.getFieldHeight(),0, 0);
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

    //Prepare the expansion hubs for bulk reads
    robot.initializeExpansionHubsForBulkRead(hardwareMap);

    //Write telemetry
    telemetry.clearAll();
    telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
    telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    if(debugOn && firstPassInitLoop) {
      Log.d(logTag, "Entering init_loop...");
      firstPassInitLoop = false;
    }
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    if(debugOn) Log.d(logTag, "Entering start...");

    runtime.reset();
    telemetry.clearAll();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

    //Move to the target position
    if(firstPassLoop){
      if(debugOn) Log.d(logTag, "Entering loop...");
      robot.getEbotsMotionController().moveToTargetPose(robot);
      firstPassLoop = false;
    }

    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Current Position: ",robot.getActualPose().toString());
    telemetry.addData("Error: ", robot.getPoseError().toString());
    telemetry.update();
  }
}
