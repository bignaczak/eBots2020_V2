package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.String.format;

@Config
@TeleOp
//@Disabled
public class TestVelocityControl extends LinearOpMode {

    org.firstinspires.ftc.teamcode.Robot robot;

    final boolean debugOn = true;
    final String logTag = "EBOTS";

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        //Configure FtcDashboard telemetry
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        robot = new Robot();
        robot.setAlliance(Alliance.RED);

        robot.initializeManipMotors(this.hardwareMap);

        waitForStart();

        Gamepad gp = gamepad2;
        DcMotorEx crane = robot.getCrane();
        crane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crane.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int cranePos;
        while(opModeIsActive()) {


            if(gp.a){
                unfoldCrane(crane);
            }

            // get the controller input for crane
            double craneInput = 0;
            if(gp.dpad_up) {
                craneInput = 1;
            } else if(gp.dpad_down){
                craneInput = -1;
            }

            cranePos = crane.getCurrentPosition();

            //Try and control the crane using velocity control
            if(Math.abs(craneInput) != 0){
                boolean allowUpwardsTravel = cranePos > 125;        //only allow upwards travel if greater than 125
                boolean requestingUpwardsTravel = Math.signum(craneInput) == 1;
                double passPower = (requestingUpwardsTravel && !allowUpwardsTravel) ? 0 : -0.2; // Pass power unless requesting upward travel when not allowed

                // if requesting downward travel, and want to go slow at end
                boolean allowDownwardTravel = cranePos < 155;
                if(!requestingUpwardsTravel){
                    if (!allowUpwardsTravel) passPower = 0.6;  //Apply high power while unfolding
                    else if(!allowDownwardTravel) passPower = 0;        //No power after encoder hits 160;
                    else passPower = 0.1;
                }

                telemetry.addData("passPower", passPower);
                crane.setPower(passPower);
            } else{
                crane.setPower(0);
            }

//            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20,0,0,0);
//            crane.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//            crane.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            if(gp.dpad_down){
//                crane.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                crane.setVelocity(100);
//                crane.setPower(1);
//            } else if(gp.dpad_up){
//                crane.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                crane.setVelocity(-100);
//                crane.setPower(1);
//            } else{
//                crane.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                crane.setPower(0);
//            }

//            robot.handleManipInput(gamepad2);

            String f = "%.2f";
            telemetry.addData("crane PIDF Coeff", crane.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
            telemetry.addData("Crane speed / Power", String.format(f,crane.getVelocity())
                    + " / " + String.format(f,crane.getPower()));
            telemetry.addData("crane Position", crane.getCurrentPosition());
            telemetry.update();
        }


    }

    private void unfoldCrane(DcMotorEx crane){
        int cranePos = crane.getCurrentPosition();
        while(cranePos < 155 && opModeIsActive()){
            if (cranePos < 65) {
                crane.setPower(1.0);
            } else if (cranePos < 125){
                crane.setPower(0.3);
            } else {
                crane.setPower(0.1);
            }
            cranePos = crane.getCurrentPosition();
            String f = "%.2f";
            telemetry.addData("Crane  Power", String.format(f,crane.getPower()));
            telemetry.addData("crane Position", cranePos);
            telemetry.update();

        }

    }
}
