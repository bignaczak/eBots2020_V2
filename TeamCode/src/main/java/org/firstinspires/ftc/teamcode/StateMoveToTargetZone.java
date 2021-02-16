package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateMoveToTargetZone implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    // ***********   CONSTRUCTOR   ***********************
    public StateMoveToTargetZone(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.MOVE_TO_TARGET_ZONE;
        this.nextAutonStateEnum = AutonStateEnum.PLACE_WOBBLE_GOAL;

        // todo:  Make sure there is a targetPose assigned to the robot
        if(debugOn) Log.d(logTag, "From StateMoveToTargetZone -- Target Zone: " + StarterStackObservation.getObservedTarget().toString());
        //set target position
        TargetZone.Zone observedTarget = StarterStackObservation.getObservedTarget();
        TargetZone targetZone = new TargetZone(robot.getAlliance(), observedTarget);
        Pose targetPose = new Pose(targetZone.getFieldPosition(), 0);
        robot.setTargetPose(targetPose);

        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);
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
        return (robot.getEbotsMotionController().isTargetPoseReached(robot)
                | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        robot.stop();
    }

    @Override
    public void performStateActions() {
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        //report telemetry
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
        opMode.telemetry.update();
    }
}
