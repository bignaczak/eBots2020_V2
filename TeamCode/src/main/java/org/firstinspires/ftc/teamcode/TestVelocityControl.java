package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.String.format;

@TeleOp
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

        while(opModeIsActive()) {
            robot.handleManipInput(gamepad2);

            String f = "%.2f";
            telemetry.addData("Shooter Power/Velocity:  ",
                    format(f, robot.getMotorPower(robot.getLauncher())) + " / "
                            + format(f, robot.getMotorVelocity(robot.getLauncher())));
            telemetry.addData("Crane Position", robot.getCrane().getCurrentPosition());

            telemetry.addData("Crane Power/Velocity:  ",
                    format(f, robot.getMotorPower(robot.getCrane())) + " / "
                            + format(f, robot.getMotorVelocity(robot.getCrane())));

            telemetry.addData("ringFeeder Position", robot.getRingFeeder().getPosition());
            telemetry.addData("ringFeeder cycle timer:", robot.getRingFeederCycleTimer().getElapsedTimeMillis());
            telemetry.update();
        }


    }
}
