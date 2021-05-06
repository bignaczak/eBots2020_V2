package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateMoveToSecondStartLine implements AutonState{

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
    public StateMoveToSecondStartLine(LinearOpMode opModeIn, Robot robotIn){
        Log.d(logTag, "Instantiating StateMoveToSecondStartLine");
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.MOVE_TO_SECOND_START_LINE;
        //TODO:  UPDATE THIS
        this.nextAutonStateEnum = AutonStateEnum.PICKUP_SECOND_WOBBLE_GOAL;

        //set target position
        Log.d(logTag, "Creating startline...");
        StartLine startline = new StartLine(StartLine.LinePosition.OUTER, robot.getAlliance());
        Log.d(logTag, "Getting field positions for start line...");
        double targetXCoord = startline.getFieldPosition().getxPosition();
        Log.d(logTag, "startline center (expect -60): " + String.format("%.2f", targetXCoord)); //Should be -60

        targetXCoord += (startline.getSizeCoordinate(CsysDirection.X)/2);  //to get to end of start line
        Log.d(logTag, "startline Size in X direction (expect 24): " + String.format("%.2f", startline.getSizeCoordinate(CsysDirection.X))); //Should be -60
        targetXCoord += 8;  // for wobble goal diameter
        targetXCoord += robot.getSizeCoordinate(CsysDirection.Y)/2;  // for robot width
        targetXCoord += 6; // crane reach


        double targetYCoord = startline.getFieldPosition().getyPosition();
        targetYCoord -= 4; // for crane placement fron robot center

        Log.d(logTag, "About to create target pose...");

        Pose targetPose = new Pose(targetXCoord, targetYCoord, 0);
        robot.setTargetPose(targetPose);

        if(debugOn){
            Log.d(logTag, "Entering state: " + currentAutonStateEnum);
            Log.d(logTag, "Actual " + robot.getActualPose().toString());
            Log.d(logTag, "Target " + robot.getTargetPose().toString());
            Log.d(logTag, robot.getPoseError().toString());
            // Compare the heading from the actual reading to that of the gyro
            robot.bulkReadSensorInputs(stateStopWatch.getElapsedTimeMillis(),false,false);
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
        if(debugOn) Log.d(logTag, "Current Time " + stateStopWatch.getElapsedTimeMillis() +
                " time limit: " + stateTimeLimit);

        boolean isCurrentPoseCorrect = robot.getEbotsMotionController().isTargetPoseReached(robot);
        boolean shouldExit = robot.getEbotsMotionController().isTargetPoseSustained(robot, timeInCorrectPosition, isCurrentPoseCorrect, targetPoseAchieved);
        targetPoseAchieved = isCurrentPoseCorrect;

        return (shouldExit | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        //Wait for user feedback before continuing
        robot.stop();

//        EbotsRev2mDistanceSensor backSensor = null;
//        for(EbotsRev2mDistanceSensor distanceSensor: robot.getEbotsRev2mDistanceSensors()){
//            if(distanceSensor.getRobotSide() == RobotSide.BACK){
//                backSensor = distanceSensor;
//            }
//        }
//
//        while(!(opMode.gamepad1.left_bumper && opMode.gamepad1.x) && opMode.opModeIsActive()){
//            backSensor.setDistanceInches();
//            opMode.telemetry.addData("Distance to Back", backSensor.getDistanceInches());
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

        //report telemetry
//        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
//        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
//        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
//        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
//        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
//        opMode.telemetry.update();
    }
}
