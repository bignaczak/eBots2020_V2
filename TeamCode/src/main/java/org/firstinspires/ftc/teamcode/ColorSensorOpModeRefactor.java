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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: Refactored", group = "Concept")
//@Disabled
public class ColorSensorOpModeRefactor extends OpMode {


  public Alliance alliance;
  private ArrayList<EbotsColorSensor> ebotsColorSensors;
  private ElapsedTime runtime = new ElapsedTime();


  @Override
  public void init() {
    ebotsColorSensors = new ArrayList<>();
    alliance = Alliance.RED;
    for (EbotsColorSensor.SensorLocation sensorLocation : EbotsColorSensor.SensorLocation.values()) {
      EbotsColorSensor sensor = new EbotsColorSensor(sensorLocation, hardwareMap);
      ebotsColorSensors.add(sensor);
    }

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
    for (EbotsColorSensor ebotsColorSensor: ebotsColorSensors){
      ebotsColorSensor.setColorValue();
    }
    telemetry.addData("Status", "Run Time: " + runtime.toString());


    //website for color: https://www.color-blindness.com/color-name-hue/ website for color hue
    for (EbotsColorSensor.TapeColor tc : EbotsColorSensor.TapeColor.values()) {
      for (EbotsColorSensor sensor : ebotsColorSensors) {
        telemetry.addData(sensor.sensorLocation.toString() + " detects" + tc.toString() + ": ", sensor.isColor(tc));

      }
    }
    for (EbotsColorSensor.TapeColor tc : EbotsColorSensor.TapeColor.values()) {
      for (RobotSide rs : RobotSide.values()) {
        telemetry.addData(rs.toString() + " detects" + tc.toString() + ": ", EbotsColorSensor.isSideOnColor(ebotsColorSensors,rs,tc));
      }
    }

    EbotsColorSensor.TapeColor launchLineColor = EbotsColorSensor.TapeColor.WHITE;
      if (EbotsColorSensor.isSideOnColor(ebotsColorSensors, RobotSide.FRONT, launchLineColor)){
        telemetry.addData("Movement:", " None");
      } else if (EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_LEFT, ebotsColorSensors).isColor(launchLineColor)){
        telemetry.addData("Movement:"," Rotate positive heading angle");
      } else if (EbotsColorSensor.getEbotsColorSensor(EbotsColorSensor.SensorLocation.FRONT_RIGHT, ebotsColorSensors).isColor(launchLineColor)){
        telemetry.addData("movement:"," Rotate negative heading angle");
      } else {
        telemetry.addData("Movement:"," Forward");
      }

      EbotsColorSensor.TapeColor startLineColor = EbotsColorSensor.TapeColor.BLUE;
      RobotSide startLineSide = RobotSide.RIGHT;
      if (alliance == Alliance.RED){
        startLineColor = EbotsColorSensor.TapeColor.RED;
        startLineSide = RobotSide.LEFT;
      }

      if (EbotsColorSensor.isSideOnColor(ebotsColorSensors, startLineSide, startLineColor)){
        telemetry.addData("Is setup correct:", " true");
      } else {
        telemetry.addData("Is setup correct: "," false");
      }

    telemetry.update();
  }
}