package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateMoveToTargetZoneAgain implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch = new StopWatch();
    long previousLoopEnd;
    int loopCount;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    StopWatch timeInCorrectPosition = new StopWatch();
    boolean targetPoseAchieved = false;


    // ***********   CONSTRUCTOR   ***********************
    public StateMoveToTargetZoneAgain(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.MOVE_TO_TARGET_ZONE_AGAIN;
        this.nextAutonStateEnum = AutonStateEnum.PARK_ON_LAUNCH_LINE;

        //set target position
        TargetZone.Zone observedTarget = StarterStackObservation.getObservedTarget();
        TargetZone targetZone = new TargetZone(robot.getAlliance(), observedTarget);
        Pose targetPose = new Pose(targetZone.getFieldPosition(), 0);
        robot.setTargetPose(targetPose);
        double craneXOffset = 5.25;
        double craneYOffset = -4.5;

        double targetZoneQ1XCenter = 6.0;
        double targetZoneQ1YCenter = 18;

        double xOffset = -targetZoneQ1XCenter - craneXOffset;
        double yOffset = targetZoneQ1YCenter + craneYOffset;

        Pose offsetTargetPose = new Pose(targetPose.getX() + xOffset, targetPose.getY() + yOffset,
                targetPose.getHeadingDeg());
        robot.setTargetPose(offsetTargetPose);
        if(debugOn){
            Log.d(logTag, "Entering state: " + currentAutonStateEnum);
            Log.d(logTag, "Actual " + robot.getActualPose().toString());
            Log.d(logTag, "Target " + robot.getTargetPose().toString());
            // Compare the heading from the actual reading to that of the gyro
            robot.bulkReadSensorInputs(stateStopWatch.getElapsedTimeMillis(),false,false);
            Log.d(logTag, "Gyro Reading: " + robot.getActualPose().getNewHeadingReadingDeg());
            for(EncoderTracker e: robot.getEncoderTrackers()){
                Log.d(logTag, e.toString());
            }

        }
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
        boolean isCurrentPoseCorrect = robot.getEbotsMotionController().isTargetPoseReached(robot);
        boolean shouldExit = robot.getEbotsMotionController().isTargetPoseSustained(robot, timeInCorrectPosition, isCurrentPoseCorrect, targetPoseAchieved);
        targetPoseAchieved = isCurrentPoseCorrect;

        return (shouldExit | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);

//        return (robot.getEbotsMotionController().isTargetPoseReached(robot)
//                | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        robot.stop();
//        while(!(opMode.gamepad1.left_bumper && opMode.gamepad1.x) && opMode.opModeIsActive()){
//            //just wait
//            opMode.telemetry.addLine("Push L_Bumper + x to exit");
//            opMode.telemetry.update();
//        }

    }

    @Override
    public void performStateActions() {
        loopCount++;
        long currentTimeMillis = stateStopWatch.getElapsedTimeMillis();
        long loopDuration = currentTimeMillis - previousLoopEnd;
        previousLoopEnd = currentTimeMillis;

        if(debugOn){
            Log.d(logTag, "Actual " + robot.getActualPose().toString());
            Log.d(logTag, stateStopWatch.toString(loopCount, loopDuration));
            //Log.d(logTag, "Target " + robot.getTargetPose().toString());
        }
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        robot.moveCraneToDragWobbleGoal();

        //report telemetry
//        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
//        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
//        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
//        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
//        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
//        opMode.telemetry.update();
    }
}
