package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *   CLASS:     DriveWheel
 *   INTENT:    Object used by a robot that contains configuration information
 *              used in performing drive calculations for mecanum drive
 */
public class DriveWheel {

    /**
     * CLASS VARIABLES
     */
    private double wheelAngleRad;
    private double wheelDiameter;
    private DcMotor wheelMotor;
    private double calculatedPower;
    //private RobotSide robotSide;
    private WheelPosition wheelPosition;

    /**
     * ENUMERATIONS
     */
    public enum WheelPosition{
        //This enumeration captures the wheel configuration information so it can be referenced during
        //construction of a DriveWheel object
        FRONT_LEFT (45, "frontLeft", RobotSide.LEFT),
        FRONT_RIGHT (-45, "frontRight", RobotSide.RIGHT),
        BACK_LEFT (-45, "backLeft", RobotSide.LEFT),
        BACK_RIGHT (45, "backRight", RobotSide.RIGHT);

        private double wheelAngleRadEnum;   //Note:  Wheel angle value stored as radians (multiply by 180/pi for degrees)
        private String motorName;
        private DcMotor.Direction motorDirectionEnum;
        private RobotSide robotSide;

        WheelPosition(double wheelAngleInDegrees, String motorName, RobotSide robotSide){
            this.wheelAngleRadEnum = Math.toRadians(wheelAngleInDegrees);
            this.motorName = motorName;
            this.robotSide = robotSide;
        }

        public double getWheelAngleRadEnum(){
            return this.wheelAngleRadEnum;
        }

        public String getMotorName(){
            return this.motorName;
        }

        public DcMotor.Direction getMotorDirectionEnum(){
            return this.motorDirectionEnum;
        }

        public RobotSide getRobotSide(){
            return this.robotSide;
        }
    }

    public enum RobotSide{
        RIGHT,
        LEFT
    }

    /**
     * CONSTRUCTORS
     */
    public DriveWheel(WheelPosition wheelPosition, HardwareMap hardwareMap){
        this.wheelPosition = wheelPosition;
        this.wheelDiameter = 4;     //Wheel diameter in inches
        this.wheelAngleRad = wheelPosition.getWheelAngleRadEnum();
        this.wheelMotor = initializeDriveMotor(wheelPosition, hardwareMap);
        this.calculatedPower = 0;
        //this.robotSide = wheelPosition.getRobotSideEnum();
    }

    /**
     * GETTERS
     */
    public double getCalculatedPower(){
        return this.calculatedPower;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public DcMotor getWheelMotor(){
        return wheelMotor;
    }

    public int getEncoderClicks(){
        DcMotor motor = getWheelMotor();
        return motor.getCurrentPosition();

    }

    public WheelPosition getWheelPosition(){
        return wheelPosition;
    }


    /**
     * INSTANCE METHODS
     */
    public DcMotor initializeDriveMotor(WheelPosition wheelPosition, HardwareMap hardwareMap){
        String motorName = wheelPosition.getMotorName();
        DcMotor myMotor = hardwareMap.get(DcMotor.class, motorName);

        //Reverse the directions if on the Right side (Uses RobotSide Enumeration)
        if(wheelPosition.getRobotSide()==RobotSide.RIGHT){
            myMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        //Reset encoders and set run mode
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return myMotor;
    }

    public void setCalculatedPower(DriveCommand driveCommand){
        //Start by analyzing the translation component of driveCommand
        double calcAngleRad = this.wheelAngleRad - driveCommand.angleRad;
        this.calculatedPower = driveCommand.magnitude * Math.cos(calcAngleRad);

        //Now apply the spin component of driveCommand
        this.applySpinToCalculatedPower(driveCommand);
    }

    private void applySpinToCalculatedPower(DriveCommand driveCommand) {
        // Positive spin means to turn to the right
        // So add power to the left motors and subtract from the right

        //This variable figures out which robotSide that the spin will be added to
        RobotSide addToRobotSide = (driveCommand.spin >=0) ? RobotSide.LEFT : RobotSide.RIGHT;
        RobotSide currentRobotSide = this.wheelPosition.getRobotSide();
        double spinMagnitude = Math.abs(driveCommand.spin); //Get the absolute value
        double spinSign = (currentRobotSide == addToRobotSide) ? 1.0 : -1.0;  //pick sign based on wheel location and drive command
        double spinPower = spinSign * spinMagnitude;        //Apply the sign
        calculatedPower = calculatedPower + spinPower;      //Update the calculated power
    }

    public void scaleCalculatedPower(double scaleFactor){
        calculatedPower = calculatedPower * scaleFactor;
    }

    public void setMotorPower(){
        wheelMotor.setPower(calculatedPower);
    }
}
