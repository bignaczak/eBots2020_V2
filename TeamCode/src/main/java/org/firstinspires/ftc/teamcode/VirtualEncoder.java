package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static java.lang.String.format;

public class VirtualEncoder {
    private int clicks;

    private static final double clicksPerInch = 869.2;
    private static final double topSpeed = 50.0;  // inches/s
    private static final double angularTopSpeed = 276.92;  //  degrees / s
    private static final double virSpinRadius = 6.0;   //  inches from spin center
    private static String debugTag = "E-Bots_VirtualEncoder";

    public VirtualEncoder(){
        clicks = 0;
    }

    public void simulateLoopOutput(DriveCommand driveCommand, EncoderTracker encoderTracker, long loopDuration){
        boolean debugOn = false;
        String logTag = "EBots_simLoopOut";

        this.processTranslationLoopOutput(driveCommand,encoderTracker,loopDuration);

        //Then apply rotation
        this.processSpinLoopOutput(driveCommand,encoderTracker, loopDuration);
    }

    private void processTranslationLoopOutput(DriveCommand driveCommand, EncoderTracker encoderTracker, long loopDuration){
        boolean debugOn = false;
        String logTag = "EBots_TransLoopOut";

        double distance = calculateSimulatedDistance(driveCommand.getMagnitude(), loopDuration);
        double robotDriveAngleRad = driveCommand.getAngleRad();
        double distanceComponent;

        if(encoderTracker.getRobotOrientation() == RobotOrientation.FORWARD){
            distanceComponent = distance * Math.cos(robotDriveAngleRad);
        } else {
            distanceComponent = distance * Math.sin(robotDriveAngleRad);
        }

        int translationClicks = (int) Math.round(distanceComponent * clicksPerInch);
        this.clicks += translationClicks;
        if (debugOn) {
            Log.d(logTag, "Added translation output" + (translationClicks) + " to " + encoderTracker.getRobotOrientation().name() + " encoder");
            Log.d(logTag, format("%.3f", distance) + " in total, " + format("%.3f",distanceComponent) +
                    "in component for " + encoderTracker.getRobotOrientation().name() + " encoder");
        }
    }

    private void processSpinLoopOutput(DriveCommand driveCommand, EncoderTracker encoderTracker, long loopDuration){
        boolean debugOn = false;
        String logTag = "EBots_SpinLoopOut";

        double spinDistance = calculateSimulatedRotation(driveCommand.getSpin(), loopDuration);  //Can be negative
        int spinClicks = (int) Math.round(spinDistance * clicksPerInch);
        this.clicks += spinClicks;

        if (debugOn) {
            Log.d(logTag, "Added Spin Clicks" + (spinClicks) + " to " + encoderTracker.getRobotOrientation().name() + " encoder");
        }
    }


    public int getClicks(){
        return this.clicks;
    }

    public static double calculateSimulatedDistance(double driveSignal, long timeStepMillis){
        double actualSpeed = driveSignal * topSpeed;
        double distance = actualSpeed * (timeStepMillis / 1000.0);
        return distance;
    }

    public static Double calculateSimulatedRotation(double spinSignal, long timeStepMillis){
        boolean debugOn = false;
        String logTag = "E-Bots_calcSimRot";
        double actualAngularSpeed = spinSignal * angularTopSpeed;
        double rotationAngle = actualAngularSpeed * (timeStepMillis / 1000.0);
        double rotationDistance = virSpinRadius * Math.toRadians(rotationAngle);
        if (debugOn) Log.d(logTag, "With spin signal " + format("%.2f", spinSignal) +
                " rotation distance of " + format("%.2f", rotationDistance) + " output" +
                " which equates to an angle of " + format("%.2f", rotationAngle));
        return rotationDistance;
    }
}
