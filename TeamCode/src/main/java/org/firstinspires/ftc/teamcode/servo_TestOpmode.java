package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp
@Disabled
public class servo_TestOpmode extends LinearOpMode {

    Servo jawServo;
    DcMotorEx craneRotate;

    final double OPEN_JAW = 0.75;
    final double CLOSE_JAW = 0.56;

    final int CARRY = -72;
    final int FOLDED = -130;
    final int GRAB = 0;

    public double motorPower = 1.0;

    public static double P;
    public static double I;
    public static double D;
    //p=10.000000 i=0.050003 d=0.000000  f=0.000000

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        jawServo = hardwareMap.get(Servo.class, "jaw");
        craneRotate = (DcMotorEx) hardwareMap.get(DcMotor.class, "crane");

        craneRotate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        craneRotate.setTargetPosition(GRAB);
        craneRotate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        craneRotate.setPower(0);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1.5,0.050003,0,0);
        craneRotate.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);

        waitForStart();

        telemetry.clearAll();
        while(opModeIsActive()) {

            if(gamepad2.a) {
                jawServo.setPosition(CLOSE_JAW);
            } else if(gamepad2.y){
                jawServo.setPosition(OPEN_JAW);
            } else if(gamepad2.dpad_up){
                craneRotate.setTargetPosition(FOLDED);
                craneRotate.setPower(motorPower);
            } else if(gamepad2.dpad_down){
                craneRotate.setTargetPosition(GRAB);
                craneRotate.setPower(motorPower);
            } else if(gamepad2.dpad_left || gamepad2.dpad_right){
                craneRotate.setTargetPosition(CARRY);
                craneRotate.setPower(motorPower);
            }
            telemetry.addLine("PID: " + craneRotate.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).toString());
            telemetry.addData("targetPosition", craneRotate.getTargetPosition());
            telemetry.addData("crane motor Power", craneRotate.getPower());
            telemetry.addData("crane motor actualPosition", craneRotate.getCurrentPosition());
            telemetry.update();
        }

    }
}
