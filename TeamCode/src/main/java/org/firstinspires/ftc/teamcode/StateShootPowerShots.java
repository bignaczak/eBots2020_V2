package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateShootPowerShots implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;

    // ***********   CONSTRUCTOR   ***********************
    public StateShootPowerShots(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.SHOOT_POWER_SHOTS;
        this.nextAutonStateEnum = AutonStateEnum.MOVE_TO_SECOND_START_LINE;
        stateTimeLimit = 1000;
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
        Log.d("EBOTS", "StateShootPowerShotes::areExitConditionsMet");
        return (stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        Log.d("EBOTS", "StateShootPowerShotes::performStateSpecificTransitionActions");

        //Create a new target pose on the launch line in the center of field
//        double xCoord = (new LaunchLine()).getX() - (robot.getSizeCoordinate(CsysDirection.X) / 2);
//        Pose targetPose = new Pose(xCoord, 36, 0);
//        robot.setTargetPose(targetPose);
    }

    @Override
    public void performStateActions() {
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        //report telemetry
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
        opMode.telemetry.update();
    }
}
