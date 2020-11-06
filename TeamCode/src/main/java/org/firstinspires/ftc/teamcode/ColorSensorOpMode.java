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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

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
  public boolean isWhite = false;
  public boolean isBlue = false;
  public boolean isRed = false;
  public Alliance alliance;
  private ColorSensor sensorColor;
  private DistanceSensor sensorDistance;
  private int sensorColorBlue;
  private int sensorColorRed;
  private int sensorColorGreen;
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
    sensorColor = hardwareMap.get(ColorSensor.class, "Color Sensor");
    sensorDistance = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
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
    int redOther = 20;
    int blueThreshold = 170;
    int blueOther = 20;
    //running until Other = false & white = True
    while (!isWhite) {

      // then cast it back to int (SCALE_FACTOR is a double)
      sensorColorRed = (int) (sensorColor.red() * SCALE_FACTOR);
      sensorColorBlue = (int) (sensorColor.blue() * SCALE_FACTOR);
      sensorColorGreen = (int) (sensorColor.green() * SCALE_FACTOR);

      //website for color: https://www.color-blindness.com/color-name-hue/ website for color hue
      //White >= 200 all
      //Red > 20B, > 130R, > 20G
      //Blue > 170B, > 20R, > 20G
      if (sensorColorBlue >= whiteThreshold && sensorColorRed >= whiteThreshold && sensorColorGreen >= whiteThreshold) {
        isWhite = true;
      }
    }

    if(alliance == Alliance.RED) {
      if (sensorColorBlue >= redOther && sensorColorRed >= redThreshold && sensorColorGreen >= redOther) {
        isRed = true;
      }

    } else {
      if (sensorColorBlue >= blueThreshold && sensorColorRed >= blueOther && sensorColorGreen >= blueOther) {
        isBlue = true;
      }
    }
        telemetry.addData("is launch line detected:",isWhite);
        telemetry.addData("is blue detected:",isBlue);
        telemetry.addData("is red detected:",isRed);
        telemetry.update();
  }
}
