package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Formatter;

import static java.lang.String.format;

public class EncoderTracker {
    /**
     *   CLASS:     EncoderTracker
     *   INTENT:    EncoderTracker records and computes the differences in encoder positions
     *              and translates them back into x,y (and maybe heading eventually)
     *              Note: This handles use cases for:
     *                  --2 encoders or 3 encoders
     *                  --Real or Virtual encoders
     */


    /***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88*/

    private DcMotorEx motor;                      //motor that the encoder is attached to
    private RobotOrientation robotOrientation;  //orientation of the encoder relative to robot reference frame
    //  FORWARD
    //  LATERAL -> Left is positive

    private int currentClicks;              //  Number of clicks for current iteration
    private int newReading;                 //  Variable for storing new encoder click reading during control cycle
                                            //  This is compared to currentClicks to determine movement

    private double spinRadius;              //  Distance from encoder to robot's spin center

    private int cumulativeClicks;                //  Cumulative Number of Clicks
    private double cumulativeDistance;               //  Total Distance traveled in inches
    private boolean isVirtual;                  //Determines whether Encoder is Real or Virtual (Virtual encoder can be used for debugging motion controller logic)
    private double wheelDiameter;
    private double clicksPerInch;   //  Clicks per inch of travel

    /**
     */

    private SpinBehavior spinBehavior;          //
    private ClickDirection clickDirection;
    private EncoderModel encoderModel;

    private final String logTag = "EBOTS";

    /***************************************************************88
     //******    STATIC VARIABLES
     //****************************************************************/

    //A list of all the encoders being tracked

    private static ArrayList<EncoderTracker> encoders = new ArrayList<>();

    /***************************************************************88
     //******    GETTERS
     //****************************************************************/
    public SpinBehavior getSpinBehavior() {
        return spinBehavior;
    }
    public RobotOrientation getRobotOrientation() {
        return robotOrientation;
    }
    public boolean getIsVirtual(){return this.isVirtual;}
    /***************************************************************88
    //******    ENUMERATIONS
    //****************************************************************/

    public enum SpinBehavior{
        /**
         * Describes whether the encoder count increases or decreases with positive angle change
         * Note:  Since positive angle is when the robot is spinning CCW when viewed from the top
         *        Encoder wheels on the right side of the robot should increase with angle,
         *        while encoder wheels on the left decrease with angle
         */
        INCREASES_WITH_ANGLE
        , DECREASES_WITH_ANGLE
    }

    public enum ClickDirection{
        /**
         * Describes whether the click count increases or decreases with positive x/y travel (forward/left travel)
         */
        STANDARD,
        REVERSE
    }

    /****************************************************************
    //******    CONSTRUCTORS
    //***************************************************************/

    public EncoderTracker(){
        this.cumulativeDistance = 0.0;
        this.cumulativeClicks = 0;
        this.newReading = 0;
        this.wheelDiameter = 3.0;
        this.spinRadius = 6.0;
        this.spinBehavior = SpinBehavior.INCREASES_WITH_ANGLE;
        this.clickDirection = ClickDirection.STANDARD;
        this.encoderModel = EncoderModel.REV;
        this.setClicksPerInch();
        this.isVirtual = false;
        this.robotOrientation = RobotOrientation.FORWARD;
        this.motor = null;
    }

    public EncoderTracker(DcMotorEx motor, RobotOrientation robotOrientation, EncoderModel encoderModel){
        this();
        //Set the motor variable and reset the encoder count
        this.setMotorAndZeroEncoder(motor);
        this.robotOrientation = robotOrientation;
        this.encoderModel = encoderModel;
        this.setClicksPerInch();        //must be reset if encoder model changes
    }

    public EncoderTracker(boolean isVirtual, RobotOrientation robotOrientation){
        this();
        this.isVirtual = true;
        this.robotOrientation = robotOrientation;
    }

    /***************************************************************
    //******    STATIC METHODS
    //****************************************************************/



    /***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //****************************************************************/

    public int getCurrentClicks(){
        return this.currentClicks;
    }

    public void reverseClickDirection(){
        this.clickDirection = ClickDirection.REVERSE;
    }

    public void setSpinBehavior(SpinBehavior spinBehaviorIn){
        this.spinBehavior = spinBehaviorIn;
    }

    public double getSpinRadius(){return this.spinRadius;}

    public void setSpinRadius(double radius){
        this.spinRadius = radius;
    }

    public void setWheelDiameter(double diam){
        this.wheelDiameter = diam;
    }

    public void setClickDirection(ClickDirection clickDirection){
        this.clickDirection = clickDirection;
    }

    public double getClicksPerInch() {return clicksPerInch;}

    /***************************************************************
     //******    CLASS METHODS
     //****************************************************************/

    private void setMotorAndZeroEncoder(DcMotorEx motor){
        //During setting of encoders, this makes sure that the encoders are zeroed
        //It first sets the encoder class variable for motor to the passed argument
        //Then reads in the runmode, resets the encoders, and sets the runmode back to orignial value

        this.motor = motor;
        DcMotorEx.RunMode incomingMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(incomingMode);
    }

    public void setNewReading(){
        //This should occur as part of a bulk reading operation and should only occur once per control cycle
        // which requires expansion hub cache setting to be set to AUTO or MANUAL

        // Since this is a hardware read, do not perform if isVirtual is true
        if (!this.isVirtual) {
            this.newReading = this.motor.getCurrentPosition();
        }
    }


    public int getIncrementalClicks(){
        //Note:  Current clicks is the value at the end of the previous loop
        //       readEncoderValue provides the new value to see how much movement has occurred since

        return (this.newReading - this.currentClicks);

    }

    private void setClicksPerInch(){
        this.clicksPerInch = encoderModel.getClicksPerRevolution() / (wheelDiameter * Math.PI);   //Circumferential wheel distance divided by clicks per revolution
    }

    public void updateEncoderCurrentClicks(){
        boolean debugOn = false;
        String logTag = "EBOTS";

        int oldValue = this.currentClicks;
        this.currentClicks = this.newReading;
        if (debugOn) Log.d(logTag, this.robotOrientation.name() + " encoder was " + oldValue
                + " and is now " + this.currentClicks);
    }

    //*************************************************************************
    //  FOR VIRTUAL ENCODERS
    //*************************************************************************

    public void simulateLoopOutput(Robot robot, long loopDuration){
        boolean debugOn = false;
        String logTag = "EBOTS";
        if(debugOn) Log.d(logTag, "Entering EncoderTracker.simulateLoopOutput...");

        this.processTranslationLoopOutput(robot,loopDuration);
        if (debugOn) {
            StringBuilder sb = new StringBuilder();
            sb.append("Back in EncoderTrack.simulateLoopOutput\n");
            sb.append(this.toString());
            Log.d(logTag, sb.toString());
        }

        //Then apply rotation
        this.processSpinLoopOutput(robot, loopDuration);
    }

    private void processTranslationLoopOutput(Robot robot, long loopDuration){
        boolean debugOn = false;
        String logTag = "EBOTS";
        if(debugOn) {
            Log.d(logTag, "Entering EncoderTracker.processTranslationLoopOutput...");
            Log.d(logTag, this.toString());
        }


        DriveCommand driveCommand = robot.getDriveCommand();
        if(debugOn) Log.d(logTag, driveCommand.toString());

        double distance = calculateSimulatedDistance(robot, loopDuration);
        double robotDriveAngleRad = driveCommand.getDriveAngleRad();  //robotDriveAngle uses the robot's reference frame
        double distanceComponent;

        if(this.robotOrientation == RobotOrientation.FORWARD){
            distanceComponent = distance * Math.cos(robotDriveAngleRad);
        } else {
            distanceComponent = distance * Math.sin(robotDriveAngleRad);
        }
        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("For encoder " + this.robotOrientation);
            sb.append(" and robot angle ");
            Formatter fmt = new Formatter(sb);
            fmt.format("%.2f", Math.toDegrees(robotDriveAngleRad));
            sb.append("° distance component = ");
            fmt.format("%.2f", distanceComponent);
            sb.append(" in");
            Log.d(logTag, sb.toString());
        }
        int translationClicks = (int) Math.round(distanceComponent * clicksPerInch);

        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("Clicks per Inch: ");
            Formatter fmt = new Formatter(sb);
            fmt.format("%.2f", clicksPerInch);
            sb.append(" Translation Clicks: ");
            sb.append(translationClicks);
            Log.d(logTag, sb.toString());
        }

        //Must be careful that the new
        this.newReading += translationClicks;
        if (debugOn) {
            Log.d(logTag, "Added translation output " + (translationClicks) + " clicks to " + this.getRobotOrientation().name() + " encoder");
            Log.d(logTag, this.toString());
        }
    }

    public double calculateSimulatedDistance(Robot robot, long timeStepMillis){
        boolean debugOn = false;
        if (debugOn) Log.d(logTag, "Entering EncoderTracker.calculateSimulatedDistance...");

        double driveMagnitude = robot.getDriveCommand().getMagnitude();
        double topSpeed = robot.getTopSpeed();
        double actualSpeed = driveMagnitude * topSpeed;    //Assumes uniform top speed in all directions that is linear with driveMagnitude

        double distance = actualSpeed * (timeStepMillis / 1000.0);

        if (debugOn) {
            StringBuilder sb = new StringBuilder();
            Formatter fmt = new Formatter(sb);
            sb.append("actualSpeed: ");
            fmt.format("%.2f", actualSpeed);
            sb.append(" in/s over timestep ");
            fmt.format("%.3f", (timeStepMillis / 1000.0));
            sb.append(" s for distance of ");
            fmt.format("%.3f", distance);
            sb.append(" in");
            Log.d(logTag, sb.toString());
        }
        return distance;
    }

    private void processSpinLoopOutput(Robot robot, long loopDuration){
        boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering processSpinLoopOutput...");

        double spinDistance = calculateSimulatedRotation(robot, loopDuration);  //Can be negative

        int spinClicks = (int) Math.round(spinDistance * this.clicksPerInch);
        //Note:  must add to the new reading
        this.newReading += spinClicks;

        if (debugOn) {
            Log.d(logTag, "Added Spin Clicks" + (spinClicks) + " to " +
                    this.getRobotOrientation().name() + " encoder");
        }
    }

    public double calculateSimulatedRotation(Robot robot, long timeStepMillis){
        boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering calculateSimulatedRotation...");


        double rotationAngleDeg = robot.estimateHeadingChangeDeg(timeStepMillis);
        double rotationDistance = this.spinRadius * Math.toRadians(rotationAngleDeg);

        if (debugOn) Log.d(logTag, "With spin signal " + format("%.2f", robot.getDriveCommand().getSpin()) +
                " rotation distance of " + format("%.2f", rotationDistance) + " output" +
                " which equates to an angle of " + format("%.2f", rotationAngleDeg));

        return rotationDistance;
    }

    @Override
    public String toString(){
        String outputString;
        outputString =  "Virtual: " + isVirtual + "  Orientation: " + this.robotOrientation.name()
                + " Current Clicks: " + this.currentClicks //+ "= Distance: " + format("%.2f", this.cumulativeDistance)
                + " New Reading: " + this.newReading
                + ", clickDirection: " + this.clickDirection.name()
                + ", spinBehavior: " + this.spinBehavior.name();
        return outputString;
    }

}


