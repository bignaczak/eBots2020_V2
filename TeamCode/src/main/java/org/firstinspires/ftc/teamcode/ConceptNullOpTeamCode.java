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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Alliance.RED;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "{Thomas}FirstOpMode", group = "Concept")
@Disabled
public class ConceptNullOpTeamCode extends OpMode {

  private Alliance alliance;
  private ElapsedTime runtime = new ElapsedTime();
  private Object ConceptNullOpTeamCode;

  private DigitalChannel digitalTouch;
  private RevBlinkinLedDriver.BlinkinPattern pattern;
  private RevBlinkinLedDriver revBlinkinLedDriver;
  private final int lockOutTime = 1000;

  @Override
  public void init() {
    // set the digital channel to input.
    digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
    revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    alliance = Alliance.BLUE;
    telemetry.addData("Status", "Initialized");
    telemetry.addData("Alliance", alliance.toString());
  }

  @Override
  public void init_loop() {

  }

  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    if (alliance == Alliance.BLUE) {
      pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
    } else {
      pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
    }
    revBlinkinLedDriver.setPattern(pattern);

    if (digitalTouch.getState()) {
      telemetry.addData("Digital Touch", "Is Not Pressed");
      alliance = Alliance.BLUE;
    } else {
      telemetry.addData("Digital Touch", "Is Pressed");
      alliance = RED;
    }

    if (runtime.milliseconds() >= lockOutTime && !digitalTouch.getState()){
      toggleAlliance();
      runtime.reset();
    }
  }

  private void toggleAlliance() {
    if (alliance == Alliance.RED) {
      alliance = Alliance.BLUE;
    } else {
      alliance = Alliance.RED;
    }
  }

}