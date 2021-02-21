package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateSpin360Degrees implements AutonState{
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

    double finalHeadingTarget;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    // ***********   CONSTRUCTOR   ***********************
    public StateSpin360Degrees(LinearOpMode opModeIn, Robot robotIn){
        if(debugOn) Log.d(logTag, "Entering StateSpin360Degrees::constructor...");
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.SPIN_360_DEGREES;
        this.nextAutonStateEnum = AutonStateEnum.AWAIT_USER_FEEDBACK;

        // Read in the current heading, record the value in class attribute
        // Increment targetPose by 120 to start rotation
        finalHeadingTarget = robot.getActualPose().getHeadingDeg();
        // This should spin the robot CCW
        double incrementedHeading = Pose.applyAngleBound(finalHeadingTarget + 120);

        // Set the updated targetPose
        Pose targetPose = new Pose(robot.getActualPose().getX(), robot.getActualPose().getY(), incrementedHeading);
        robot.setTargetPose(targetPose);

        //Zero all the encoders
        for(EncoderTracker e: robot.getEncoderTrackers()){
            e.zeroEncoder();
        }

        opMode.telemetry.clearAll();
        stateStopWatch = new StopWatch();
        String fmt = "%.0f";
        opMode.telemetry.addLine("Preparing to spin CCW...");
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
        //if(debugOn) Log.d(logTag, "Entering StateSpin360Degrees::areExitConditionsMet...");

        return robot.getEbotsMotionController().isTargetPoseReached(robot);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        if(debugOn) Log.d(logTag, "Entering StateSpin360Degrees::TransitionActions...");
        for(EncoderTracker e: robot.getEncoderTrackers()){
            opMode.telemetry.addLine(e.toString());
            //  To find effective radius, use s = r*theta
            //  s = clicks / clicks per rotation * (pi * e.wheelDiameter)
            //  then divide by theta, which is the headingChange in radians
            double distTraveled = (e.getCurrentClicks() / e.getClicksPerInch());
            double actualSpinAngle = 360 - (robot.getPoseError().getHeadingErrorDeg());  // includes error
            double calculatedSpinRadius = Math.abs(distTraveled / Math.toRadians(actualSpinAngle));
            e.setCalculatedSpinRadius(calculatedSpinRadius);
            opMode.telemetry.addLine("Effective Radius: " + String.format("%.2f", calculatedSpinRadius));
            if(debugOn) {
                Log.d(logTag, e.toString());
                Log.d(logTag, "distTraveled: " + String.format("%.2f", distTraveled) +
                        " actualSpinAngle: " + String.format("%.1f", actualSpinAngle) +
                        " calculatedSpinRadius: " + String.format("%.2f", calculatedSpinRadius));
            }
        }

        robot.stop();
    }

    @Override
    public void performStateActions() {
        //if(debugOn) Log.d(logTag, "Entering StateSpin360Degrees::performStateActions...");

        //look at the target pose and keep moving it clockwise until it is back to 0
        if(Math.abs(robot.getPoseError().getHeadingErrorDeg()) < 15     //if getting close to target
                && robot.getTargetPose().getHeadingDeg() != finalHeadingTarget){         //and target isn't back to zero already
            //bump the target by an additional 120 deg
            double newHeadingTarget = Pose.applyAngleBound(robot.getTargetPose().getHeadingDeg() + 120);
            robot.getTargetPose().setHeadingDeg(newHeadingTarget);
        }

        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);
        //report telemetry
        opMode.telemetry.addLine("Spinning CCW...");
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
        for(EncoderTracker e: robot.getEncoderTrackers()){
            opMode.telemetry.addLine(e.toString());
            //  To find effective radius, use s = r*theta
            //  s = clicks / clicks per rotation * (pi * e.wheelDiameter)
            //  then divide by theta, which is the headingChange in radians
            double distTraveled = (e.getCurrentClicks() / e.getClicksPerInch()) * Math.PI * e.getWheelDiameter();
            double calculatedSpinRadius = Math.abs(distTraveled / (2*Math.PI));
                opMode.telemetry.addLine("Effective Radius: " + String.format("%.2f", calculatedSpinRadius));
        }
        opMode.telemetry.update();
    }

}
