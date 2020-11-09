package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

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

    private VirtualEncoder virtualEncoder;      //Virtual encoder can be used for debugging algorithm
    private SpinBehavior spinBehavior;          //
    private ClickDirection clickDirection;
    private EncoderModel encoderModel;


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
        this.virtualEncoder = null;
        this.wheelDiameter = 3.0;
        this.spinRadius = 6.0;
        this.spinBehavior = SpinBehavior.INCREASES_WITH_ANGLE;
        this.clickDirection = ClickDirection.STANDARD;
        this.encoderModel = EncoderModel.REV;
        this.setClicksPerInch();
        this.isVirtual = false;
        this.virtualEncoder = null;
        this.robotOrientation = RobotOrientation.FORWARD;
        this.motor = null;

    }

    public EncoderTracker(DcMotorEx motor, RobotOrientation robotOrientation, EncoderModel encoderModel){
        this();
        this.motor = motor;
        this.robotOrientation = robotOrientation;
        this.encoderModel = encoderModel;
        this.setClicksPerInch();        //must be reset if encoder model changes
    }

    public EncoderTracker(boolean isVirtual, RobotOrientation robotOrientation){
        this();
        this.isVirtual = true;
        //TODO:  Is this still necessary when setting the flag?
        this.virtualEncoder = null;
    }

    /***************************************************************
    //******    STATIC METHODS
    //****************************************************************/




    public static void updateVirtualEncoders(DriveCommand driveCommand, Long loopDuration, Pose currentPose){

        for (EncoderTracker e: encoders){
            //simulate the loop event to the virtual encoder
            e.virtualEncoder.simulateLoopOutput(driveCommand, e, loopDuration);
        }
    }


    /***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //****************************************************************/


    public static Integer getEncoderTrackerCount(){
        return encoders.size();
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

    public int getCurrentClicks(){
        return this.currentClicks;
    }

    //  TODO:  Replace this with a bulk read operation
    public int setNewReading(){
        //This should occur as part of a bulk reading operation and should only occur once per control cycle
        // which requires expansion hub cache setting to be set to AUTO or MANUAL
        return this.newReading = this.motor.getCurrentPosition();
    }

    public int getIncrementalClicks(){
        //Note:  Current clicks is the value at the end of the previous loop
        //       readEncoderValue provides the new value to see how much movement has occurred since
        return this.newReading - this.currentClicks;
    }

    private void setClicksPerInch(){
        this.clicksPerInch = (wheelDiameter * Math.PI) / encoderModel.getClicksPerRevolution();   //Circumferential wheel distance divided by clicks per revolution
    }

    public void updateEncoderCurrentClicks(){
        boolean debugOn = true;
        String logTag = "EBots_upEncCurClicks";

        int oldValue = this.currentClicks;
        this.currentClicks = this.newReading;
        if (debugOn) Log.d(logTag, this.robotOrientation.name() + " encoder was " + oldValue
                + " and is now " + this.currentClicks);
    }

    @Override
    public String toString(){
        String outputString;
        outputString =  "Virtual: " + isVirtual + "  Orientation: " + this.robotOrientation.name()
                + " Clicks: " + this.currentClicks + "= Distance: " + format("%.2f", this.cumulativeDistance)
                + ", clickDirection: " + this.clickDirection.name()
                + ", spinBehavior: " + this.spinBehavior.name();
        return outputString;
    }

}


