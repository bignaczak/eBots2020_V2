package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class StateMoveForCalibration implements AutonState{
    /**
     * This class is intended to serve as a calibration tool
     * The class can be instantiated with movement instructions
     * Exit conditions require user feedback so recordings can be made
     */
    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    // ***********   CONSTRUCTOR   ***********************
    public StateMoveForCalibration(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.MOVE_FOR_CALIBRATION;
        this.nextAutonStateEnum = AutonStateEnum.AWAIT_USER_FEEDBACK;

        Pose targetPose = ((AutonEbotsV1_Calibration) opMode).getNextPose();
        if (targetPose != null) {
            robot.setTargetPose(targetPose);
        }

        opMode.telemetry.clearAll();
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
        return robot.getEbotsMotionController().isTargetPoseReached(robot);
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
