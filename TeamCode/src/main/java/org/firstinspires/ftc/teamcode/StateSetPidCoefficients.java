package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateSetPidCoefficients implements AutonState{
    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;

    StopWatch lockoutTimer = new StopWatch();
    long lockoutDuration = 750L;

    final boolean debugOn = false;
    final String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateSetPidCoefficients(LinearOpMode opModeIn, Robot robotIn) {
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Instantiating class");

        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.SET_PID_COEFFICIENTS;
        this.nextAutonStateEnum = AutonStateEnum.PREMATCH_SETUP;

        opMode.telemetry.clearAll();
        opMode.telemetry.addData("Current State: ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine("Push Left Bumper + X on Gamepad1 to proceed");
        opMode.telemetry.update();
    }


    // ***********   GETTERS   ***********************
    @Override
    public AutonStateEnum getNextAutonStateEnum() {
        return nextAutonStateEnum;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return currentAutonStateEnum;
    }



    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // No exit condition in this method because relies on
        return (opMode.gamepad1.left_bumper && opMode.gamepad1.x);
    }

    @Override
    public void performStateSpecificTransitionActions() {

    }

        @Override
    public void performStateActions() {
            // Only process signals after lockout timer
            if(lockoutTimer.getElapsedTimeMillis() > lockoutDuration){
                processGamepadInput();
            }
            updateTelemetry();
        }


    private void processGamepadInput(){
        double pAdder = 0.05;
        double iAdder = 0.025;

        Gamepad gamepad= opMode.gamepad1;
        Speed spd = robot.getEbotsMotionController().getSpeed();

        // update the coefficients based on controller input
        if(gamepad.left_bumper && gamepad.x) {
            // Do nothing if both are pressed, that is an exit condition
        } else if (gamepad.y){
            double newK_p = spd.getK_p() + pAdder;
            spd.setK_p(newK_p);
            lockoutTimer.reset();
        } else if (gamepad.a){
            double newK_p = spd.getK_p() - pAdder;
            spd.setK_p(newK_p);
            lockoutTimer.reset();
        }else if(gamepad.b){
            double newK_i = spd.getK_i() + iAdder;
            spd.setK_i(newK_i);
            lockoutTimer.reset();
        }else if(gamepad.x){
            double newK_i = spd.getK_i() - iAdder;
            spd.setK_i(newK_i);
            lockoutTimer.reset();
        } else if(gamepad.dpad_up){
            double newS_p = spd.getS_p() + pAdder;
            spd.setS_p(newS_p);
            lockoutTimer.reset();
        } else if(gamepad.dpad_down){
            double newS_p = spd.getS_p() - pAdder;
            spd.setS_p(newS_p);
            lockoutTimer.reset();
        } else if(gamepad.dpad_right){
            double newS_i = spd.getS_i() + iAdder;
            spd.setS_i(newS_i);
            lockoutTimer.reset();
        } else if(gamepad.dpad_left){
            double newS_i = spd.getS_i() - iAdder;
            spd.setS_i(newS_i);
            lockoutTimer.reset();
        }

    }

    private void updateTelemetry(){
        Telemetry t = opMode.telemetry;
        Speed spd = robot.getEbotsMotionController().getSpeed();
        String fmt = "%.2f";
        t.addData("Translate Proportional Coeff K_p: ", String.format(fmt, spd.getK_p()));
        t.addData("Translate Integral Coeff K_i: ", String.format(fmt, spd.getK_i()));
        t.addData("Spin Proportional Coeff S_p: ", String.format(fmt, spd.getS_p()));
        t.addData("Spin Integral Coeff S_i: ", String.format(fmt, spd.getS_i()));
        t.update();
    }
}
