package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static java.lang.String.format;

public class VirtualEncoder {
    //Class Instance variables
    private int clicks;
    private double wheelDiameter;
    private double clicksPerInch;
    private EncoderModel encoderModel;
    private double spinRadius = 6.0;   //  inches from spin center, position of wheel from robot spin center
    private double topSpeed;  // inches/s
    private  double angularTopSpeed = 276.92;  //  degrees / s

    //Class static variables
    private static String debugTag = "E-Bots_VirtualEncoder";

    public VirtualEncoder(){
        this.clicks = 0;
        this.encoderModel = EncoderModel.REV;
        this.wheelDiameter = 3.0;
        this.clicksPerInch = (wheelDiameter * Math.PI) / encoderModel.getClicksPerRevolution();   //Circumferential wheel distance divided by clicks per revolution
        this.topSpeed = 50.0;
        this.angularTopSpeed = 276.92;
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
        double robotDriveAngleRad = driveCommand.getDriveAngleRad();  //robotDriveAngle uses the robot's reference frame
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

    public double calculateSimulatedDistance(double driveSignal, long timeStepMillis){
        double actualSpeed = driveSignal * topSpeed;    //Assumes uniform top speed in all directions that is linear with driveSignal
        double distance = actualSpeed * (timeStepMillis / 1000.0);
        return distance;
    }

    public double calculateSimulatedRotation(double spinSignal, long timeStepMillis){
        boolean debugOn = false;
        String logTag = "E-Bots_calcSimRot";
        double actualAngularSpeed = spinSignal * angularTopSpeed;
        double rotationAngle = actualAngularSpeed * (timeStepMillis / 1000.0);
        double rotationDistance = spinRadius * Math.toRadians(rotationAngle);
        if (debugOn) Log.d(logTag, "With spin signal " + format("%.2f", spinSignal) +
                " rotation distance of " + format("%.2f", rotationDistance) + " output" +
                " which equates to an angle of " + format("%.2f", rotationAngle));
        return rotationDistance;
    }
}
