package org.firstinspires.ftc.teamcode;

import android.util.Log;

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

    double xTravel;
    double yTravel;
    double headingChange;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    // ***********   CONSTRUCTOR   ***********************
    public StateMoveForCalibration(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.MOVE_FOR_CALIBRATION;
        this.nextAutonStateEnum = AutonStateEnum.AWAIT_USER_FEEDBACK;

        if(debugOn) Log.d(logTag, "Entering StateMoveForCalibration::constructor...");
        Pose targetPose = ((AutonEbotsV1_Calibration) opMode).getNextPose();
        if (targetPose != null) {
            robot.setTargetPose(targetPose);
            if(debugOn) {
                Log.d(logTag, "Target: " + targetPose.toString());
                Log.d(logTag, "Actual: " + robot.getActualPose().toString());
                Log.d(logTag, "Error: " + robot.getPoseError().toString());
            }

        } else{
            //  Translation states are complete, set to spin next
            if(debugOn) Log.d(logTag, "No more translations, setting spin as next auton state");
            nextAutonStateEnum = AutonStateEnum.SPIN_360_DEGREES;
        }

        //Zero all the encoders
        for(EncoderTracker e: robot.getEncoderTrackers()){
            e.zeroEncoder();
        }

        opMode.telemetry.clearAll();
        stateStopWatch = new StopWatch();
        String fmt = "%.0f";
        xTravel = robot.getPoseError().getXError();
        yTravel = robot.getPoseError().getYError();
        headingChange = robot.getPoseError().getHeadingErrorDeg();
        opMode.telemetry.addLine("Travel Instructions X | Y | Heading: " +
                String.format(fmt, xTravel) + " | " +
                String.format(fmt, yTravel) + " | " +
                String.format(fmt, headingChange)
        );
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
        if(debugOn) {
            Log.d(logTag, "Entering StateMoveForCalibration::TransitionActions..." + robot.getActualPose().toString());
            Log.d(logTag, "Actual : " + robot.getActualPose().toString());
            Log.d(logTag, "Error: " + robot.getPoseError().toString());
        }
        for(EncoderTracker e: robot.getEncoderTrackers()){
            opMode.telemetry.addLine(e.toString());
            if(headingChange != 0){
                //  To find effective radius, use s = r*theta
                //  s = clicks / clicks per rotation * (pi * e.wheelDiameter)
                //  then divide by theta, which is the headingChange in radians
                double distTraveled = (e.getCurrentClicks() / e.getClicksPerInch()) * Math.PI * e.getWheelDiameter();
                double calculatedSpinRadius = Math.abs(distTraveled / Math.toRadians(headingChange));
                e.setCalculatedSpinRadius(calculatedSpinRadius);
                opMode.telemetry.addLine("Effective Radius: " + String.format("%.2f", calculatedSpinRadius));
            } else{
                e.setCalculatedSpinRadius(0);
            }
        }

    }

    @Override
    public void performStateActions() {
        //todo: Add calculations for encoder diameter and spin radius
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        //report telemetry
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addData("Is target reached? ", robot.getEbotsMotionController().isTargetPoseReached(robot));
        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
        for(EncoderTracker e: robot.getEncoderTrackers()){
            opMode.telemetry.addLine(e.toString());
            if(headingChange != 0){
                //  To find effective radius, use s = r*theta
                //  s = clicks / clicks per rotation * (pi * e.wheelDiameter)
                //  then divide by theta, which is the headingChange in radians
                double distTraveled = (e.getCurrentClicks() / e.getClicksPerInch()) * Math.PI * e.getWheelDiameter();
                double calculatedSpinRadius = Math.abs(distTraveled / Math.toRadians(headingChange));
                opMode.telemetry.addLine("Effective Radius: " + String.format("%.2f", calculatedSpinRadius));
            }
        }
        opMode.telemetry.update();
    }

}
