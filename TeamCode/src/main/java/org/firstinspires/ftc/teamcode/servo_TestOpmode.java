package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servo_TestOpmode extends LinearOpMode {

    Servo jawServo;
    DcMotorEx craneRotate;

    final double OPEN_JAW = 0.75;
    final double CLOSE_JAW = 0.56;

    final int CARRY = -72;
    final int FOLDED = -130;
    final int GRAB = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        jawServo = hardwareMap.get(Servo.class, "jaw");
        craneRotate = hardwareMap.get(DcMotorEx.class, "crane");

        craneRotate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        craneRotate.setTargetPosition(GRAB);
        craneRotate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        craneRotate.setPower(0);
        waitForStart();

        telemetry.clearAll();
        while(opModeIsActive()) {

            if(gamepad2.a) {
                jawServo.setPosition(CLOSE_JAW);
            } else if(gamepad2.y){
                jawServo.setPosition(OPEN_JAW);
            } else if(gamepad2.dpad_up){
                craneRotate.setTargetPosition(FOLDED);
                craneRotate.setPower(0.5);
            } else if(gamepad2.dpad_down){
                craneRotate.setTargetPosition(GRAB);
                craneRotate.setPower(0.5);
            } else if(gamepad2.dpad_left || gamepad2.dpad_right){
                craneRotate.setTargetPosition(CARRY);
                craneRotate.setPower(0.5);
            }
            telemetry.addData("targetPosition", craneRotate.getTargetPosition());
            telemetry.addData("crane motor Power", craneRotate.getPower());
            telemetry.addData("crane motor actualPosition", craneRotate.getCurrentPosition());
            telemetry.update();
        }

    }
}
