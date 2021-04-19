package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateAlignForSecondWobbleGoal implements AutonState {

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;
    EbotsRev2mDistanceSensor backSensor;
    EbotsRev2mDistanceSensor rightSensor;
    // ***********   CONSTRUCTOR   ***********************
    public StateAlignForSecondWobbleGoal(LinearOpMode opModeIn, Robot robotIn) {
        this.opMode = opModeIn;
        this.robot = robotIn;
        assignCorrectSensors();
        refineActualPose();
        this.currentAutonStateEnum = AutonStateEnum.ALIGN_FOR_SECOND_WOBBLEGOAL;
        this.nextAutonStateEnum = AutonStateEnum.PICKUP_SECOND_WOBBLE_GOAL;
    }

    public void assignCorrectSensors(){
        for (EbotsRev2mDistanceSensor distanceSensor : robot.getEbotsRev2mDistanceSensors()) {
            if (distanceSensor.getRobotSide() == RobotSide.BACK) {
                backSensor = distanceSensor;
            } else if (distanceSensor.getRobotSide() == RobotSide.RIGHT) {
                rightSensor = distanceSensor;
            }
        }
    }

    private void refineActualPose(){
        double yMeasurement = 0;
        double xMeasurement = 0;
        int numReadings = 10;
        for (int i = 0; i < numReadings; i++) {
            backSensor.setDistanceInches();
            rightSensor.setDistanceInches();
            yMeasurement += backSensor.getDistanceInches();
            xMeasurement += rightSensor.getDistanceInches();
        }
        xMeasurement /= numReadings;
        yMeasurement /= numReadings;
        double refinedX = -72 + xMeasurement + 8;
        double refinedY = 24 - yMeasurement - 9;
        double targetX = robot.getTargetPose().getX();
        double targetY = robot.getTargetPose().getY();
        refinedX = Math.abs(refinedX - targetX) < 12 ? refinedX : robot.getActualPose().getX();
        refinedY = Math.abs(refinedY - targetY) < 12 ? refinedY : robot.getActualPose().getY();
        robot.setActualPose(new Pose(refinedX, refinedY, robot.getActualPose().getHeadingDeg()));
    }





    @Override
    public AutonStateEnum getNextAutonStateEnum() {
        return this.nextAutonStateEnum;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return this.currentAutonStateEnum;
    }

    @Override
    public boolean areExitConditionsMet() {
        boolean isCurrentPoseCorrect = robot.getEbotsMotionController().isTargetPoseReached(robot);
        return isCurrentPoseCorrect;
    }

    @Override
    public void performStateSpecificTransitionActions() {
    robot.stop();
    }

    @Override
    public void performStateActions() {
        //todo add time limit
        //go to correct position
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
        opMode.telemetry.update();
    }
}