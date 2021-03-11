package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateTestControlLoopSpeed implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit = 50000;
    StopWatch stateStopWatch = new StopWatch();
    long previousLoopEnd;
    int loopCount;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    // ***********   CONSTRUCTOR   ***********************
    public StateTestControlLoopSpeed(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.TEST_CONTROL_LOOP_SPEED;
        this.nextAutonStateEnum = null;

        Pose targetPose = new Pose(robot.getActualPose().getX() + 36, robot.getActualPose().getY(), robot.getActualPose().getHeadingDeg());
        robot.setTargetPose(targetPose);

        stateStopWatch = new StopWatch();
    }

    // ***********   GETTERS    ***********************
    @Override
    public AutonStateEnum getNextAutonStateEnum() {
        return this.nextAutonStateEnum;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return this.currentAutonStateEnum;
    }

    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        return (stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        robot.stop();

    }

    @Override
    public void performStateActions() {
        loopCount++;
        long currentTimeMillis = stateStopWatch.getElapsedTimeMillis();
        long loopDuration = currentTimeMillis - previousLoopEnd;
        previousLoopEnd = currentTimeMillis;
        Speed speed = robot.getEbotsMotionController().getSpeed();

        robot.bulkReadSensorInputs(loopDuration,false,false);
        robot.updateActualPose();
        robot.updateAllSensorValues();
        robot.getPoseError().calculateError(robot, loopDuration, speed);
        robot.setDriveCommand(new DriveCommand(robot, speed));
        robot.calculateDrivePowers();
        robot.drive();



        //report telemetry
        opMode.telemetry.addData("Current Time", stateStopWatch.getElapsedTimeMillis());
        opMode.telemetry.addLine(stateStopWatch.toString(loopCount));
        opMode.telemetry.update();
//        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
//        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
//        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
//        opMode.telemetry.update();
    }
}
