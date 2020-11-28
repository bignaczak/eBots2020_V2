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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
@Disabled
public class ColorSensorOpMode extends OpMode {
  public boolean frontLeftIsWhite = false;
  public boolean frontRightIsWhite = false;
  public boolean backRightIsWhite = false;
  public boolean backLeftIsWhite = false;
  public boolean frontPairSensors = false;
  public boolean leftPairSensors = false;
  public boolean rightPairSensors = false;
  public boolean backPairSensors = false;

  public boolean frontLeftIsBlue = false;
  public boolean frontRightIsBlue = false;
  public boolean backLeftIsBlue = false;
  public boolean backRightIsBlue = false;

  public boolean frontLeftIsRed = false;
  public boolean frontRightIsRed = false;
  public boolean backLeftIsRed = false;
  public boolean backRightIsRed = false;

  public Alliance alliance;
  private ColorSensor backLeftSensorColor;
  private ColorSensor backRightSensorColor;
  private ColorSensor frontRightSensorColor;
  private ColorSensor frontLeftSensorColor;
  private DistanceSensor sensorDistance;
  private int frontLeftSensorColorBlue;
  private int frontLeftSensorColorRed;
  private int frontLeftSensorColorGreen;
  private int frontRightSensorColorBlue;
  private int frontRightSensorColorGreen;
  private int frontRightSensorColorRed;

  private int backRightSensorColorBlue;
  private int backRightSensorColorGreen;
  private int backRightSensorColorRed;
  private int backLeftSensorColorBlue;
  private int backLeftSensorColorRed;
  private int backLeftSensorColorGreen;

  private ElapsedTime runtime = new ElapsedTime();

  public ColorSensorOpMode(Alliance a) {
    this.alliance = a;
  }

  // hsvValues is an array that will hold the hue, saturation, and value information.
  float hsvValues[] = {0F, 0F, 0F};

  // values is a reference to the hsvValues array.
  final float values[] = hsvValues;

  // sometimes it helps to multiply the raw RGB values with a scale factor
  // to amplify/attentuate the measured values.
  final double SCALE_FACTOR = 255;

  @Override
  public void init() {
    frontRightSensorColor = hardwareMap.get(ColorSensor.class, "frontRightSensorColor");
    frontLeftSensorColor = hardwareMap.get(ColorSensor.class, "frontLeftSensorColor");
    backRightSensorColor = hardwareMap.get(ColorSensor.class, "backRightSensorColor");
    backLeftSensorColor = hardwareMap.get(ColorSensor.class, "backLeftSensorColor");
    sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    telemetry.addData("Status", "Initialized");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    int whiteThreshold = 200;
    int redThreshold = 130;
    int redOther = 40;
    int blueThreshold = 170;
    int blueOther = 40;
    //running until Other = false & white = True
    while (!frontPairSensors || !rightPairSensors || !leftPairSensors || !backPairSensors) {

      // then cast it back to int (SCALE_FACTOR is a double)
      frontLeftSensorColorRed = (int) (frontLeftSensorColor.red() * SCALE_FACTOR);
      frontLeftSensorColorBlue = (int) (frontLeftSensorColor.blue() * SCALE_FACTOR);
      frontLeftSensorColorGreen = (int) (frontLeftSensorColor.green() * SCALE_FACTOR);

      frontRightSensorColorRed = (int) (frontRightSensorColor.red() * SCALE_FACTOR);
      frontRightSensorColorBlue = (int) (frontRightSensorColor.blue() * SCALE_FACTOR);
      frontRightSensorColorGreen = (int) (frontRightSensorColor.green() * SCALE_FACTOR);

      backLeftSensorColorRed = (int) (backLeftSensorColor.red() * SCALE_FACTOR);
      backLeftSensorColorBlue = (int) (backLeftSensorColor.blue() * SCALE_FACTOR);
      backLeftSensorColorGreen = (int) (backLeftSensorColor.green() * SCALE_FACTOR);

      backRightSensorColorRed = (int) (backRightSensorColor.red() * SCALE_FACTOR);
      backRightSensorColorBlue = (int) (backRightSensorColor.blue() * SCALE_FACTOR);
      backRightSensorColorGreen = (int) (backRightSensorColor.green() * SCALE_FACTOR);


      //website for color: https://www.color-blindness.com/color-name-hue/ website for color hue
      //White >= 200 all
      //Red > 20B, > 130R, > 20G
      //Blue > 170B, > 20R, > 20G
      //front sensors
      frontLeftIsWhite = frontLeftSensorColorBlue >= whiteThreshold && frontLeftSensorColorRed >= whiteThreshold && frontLeftSensorColorGreen >= whiteThreshold;
      frontRightIsWhite = frontRightSensorColorBlue >= whiteThreshold && frontRightSensorColorRed >= whiteThreshold && frontRightSensorColorGreen >= whiteThreshold;
      //back sensors
      backRightIsWhite = backRightSensorColorBlue >= whiteThreshold && backRightSensorColorRed >= whiteThreshold && backRightSensorColorGreen >= whiteThreshold;
      backLeftIsWhite = backLeftSensorColorBlue >= whiteThreshold && backLeftSensorColorRed >= whiteThreshold && backLeftSensorColorGreen >= whiteThreshold;

      frontPairSensors = frontRightIsWhite && frontLeftIsWhite;
      leftPairSensors = backLeftIsWhite && frontLeftIsWhite;
      rightPairSensors = frontRightIsWhite && backRightIsWhite;
      backPairSensors = backRightIsWhite && backLeftIsWhite;

      //single sensors
      telemetry.addData("front left sensor detects launch line:", frontLeftIsWhite);
      telemetry.addData("front right sensor detects launch line:", frontRightIsWhite);
      telemetry.addData("back right sensor detects launch line:", backRightIsWhite);
      telemetry.addData("back left sensor detects launch line:", backLeftIsWhite);
      // paired sensors
        telemetry.addData("no movement", "is launch line detected by front sensors:", frontPairSensors);
        telemetry.addData("no movement", "is launch line detected by left sensors:", leftPairSensors);
        telemetry.addData("no movement", "is launch line detected by right sensors:", rightPairSensors);
        telemetry.addData("no movement", "is launch line detected by back sensors:", backPairSensors);


      if (!frontRightIsWhite && !frontLeftIsWhite){
        telemetry.addData("movement:", "forward");
      } else if (frontPairSensors){
        telemetry.addData("movement:","none");
      } else if (backPairSensors) {
        telemetry.addData("movement:", "none");
      }else if (leftPairSensors){
          telemetry.addData("movement:","none");
      } else if (rightPairSensors) {
          telemetry.addData("movement:", "none");
      } else if (frontRightIsWhite && !frontLeftIsWhite){
        telemetry.addData("movement:","rotate clock-wise");
      } else if (frontLeftIsWhite && !frontRightIsWhite){
        telemetry.addData("movement:","counter clock-wise");
      } else if (backRightIsWhite && !backLeftIsWhite){
        telemetry.addData("movement:","rotate clock-wise");
      } else if (!backRightIsWhite && backLeftIsWhite){
        telemetry.addData("movement:","counter clock-wise");
      } else {
        telemetry.addData("movement:","unknown");
      }
      telemetry.update();
  }

    frontLeftIsRed = frontLeftSensorColorBlue <= redOther && frontLeftSensorColorRed >= redThreshold && frontLeftSensorColorGreen <= redOther;
    frontRightIsRed = frontRightSensorColorBlue <= redOther && frontRightSensorColorRed >= redThreshold && frontRightSensorColorGreen <= redOther;
    backLeftIsRed = backLeftSensorColorBlue <= redOther && backLeftSensorColorRed >= redThreshold && backLeftSensorColorGreen <= redOther;
    backRightIsRed = backRightSensorColorBlue <= redOther && backRightSensorColorRed >= redThreshold && backRightSensorColorGreen <= redOther;


    frontLeftIsBlue = frontLeftSensorColorBlue <= blueOther && frontLeftSensorColorRed >= blueThreshold && frontLeftSensorColorGreen <= blueOther;
    frontRightIsBlue = frontRightSensorColorBlue <= blueOther && frontRightSensorColorRed >= blueThreshold && frontRightSensorColorGreen <= blueOther;
    backLeftIsBlue = backLeftSensorColorBlue <= blueOther && backLeftSensorColorRed >= blueThreshold && backLeftSensorColorGreen <= blueOther;
    backRightIsBlue = backRightSensorColorBlue <= blueOther && backRightSensorColorRed >= blueThreshold && backRightSensorColorGreen <= blueOther;

    if(alliance == Alliance.RED) {
      telemetry.addData("is red detected by front left sensor:", frontLeftIsRed);
      telemetry.addData("is red detected by front right sensor:", frontRightIsRed);
      telemetry.addData("is red detected by back left sensor:", backLeftIsRed);
      telemetry.addData("is red detected by back right sensor:", backRightIsRed);
      if (frontLeftIsRed && backLeftIsRed){
        telemetry.addData("position","correct");
      } else {
        telemetry.addData("position","incorrect");
      }
    } else {
      telemetry.addData("is blue detected by front left sensor:", frontLeftIsBlue);
      telemetry.addData("is blue detected by front right sensor:", frontRightIsBlue);
      telemetry.addData("is blue detected by back left sensor:", backLeftIsBlue);
      telemetry.addData("is blue detected by back right sensor:", backRightIsBlue);
      if (frontRightIsRed && backRightIsRed){
        telemetry.addData("position","correct");
      } else {
        telemetry.addData("position","incorrect");
      }
    }
    telemetry.update();
  }
}