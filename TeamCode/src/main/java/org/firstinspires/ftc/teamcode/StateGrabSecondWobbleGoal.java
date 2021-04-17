package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

public class StateGrabSecondWobbleGoal implements AutonState {

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;
    boolean inRightPosition = false;
    boolean inFrontPosition = false;
    double backWall = 24;
    double rightWall = 30;
    double targetXCoord = -48;
    double targetYCoord = -48;
    EbotsRev2mDistanceSensor frontSensor = null;
    EbotsRev2mDistanceSensor rightSensor = null;

    public EbotsRev2mDistanceSensor getCorrectSensor(){
        rightSensor.getDistanceInches();
        frontSensor.getDistanceInches();
        for (EbotsRev2mDistanceSensor distanceSensor : robot.getEbotsRev2mDistanceSensors()) {
            if (distanceSensor.getRobotSide() == RobotSide.FRONT) {
                frontSensor = distanceSensor;
                frontSensor.setDistanceInches();
            } else if (distanceSensor.getRobotSide() == RobotSide.RIGHT) {
                rightSensor = distanceSensor;
                rightSensor.setDistanceInches();
            }
        }
        return null;
    }

    // ***********   CONSTRUCTOR   ***********************
    public StateGrabSecondWobbleGoal(LinearOpMode opModeIn, Robot robotIn) {
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.GRAB_SECOND_WOBBLEGOAL;
        this.nextAutonStateEnum = AutonStateEnum.DELIVER_SECOND_WOBBLEGOAL;
        Pose targetPose = new Pose(targetXCoord, targetYCoord, 90);
        robot.setTargetPose(targetPose);
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
        boolean inCorrectPosition = inFrontPosition & inRightPosition;
        return inCorrectPosition;
    }

    @Override
    public void performStateSpecificTransitionActions() {
    robot.stop();
    //todo add more if needed
    }

    @Override
    public void performStateActions() {
        //go to correct position
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        //check to see if robot is in correct position
        opMode.telemetry.addData("Robot is ", frontSensor.getDistanceInches() + "from back wall");
        opMode.telemetry.addData("Robot is ", rightSensor.getDistanceInches() + "from right wall");
        if (rightSensor.getDistanceInches() == rightWall){
            inRightPosition = true;
        }else {
            inRightPosition = false;
        }
        if (frontSensor.getDistanceInches() == backWall){
            inFrontPosition = true;
        } else {
            inFrontPosition = true;
        }

        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
        opMode.telemetry.update();
    }
}