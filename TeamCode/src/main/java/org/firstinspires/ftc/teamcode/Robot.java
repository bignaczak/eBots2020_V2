package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.DriveWheel.*;

/**
 *   CLASS:     Robot
 *   INTENT:    Robot is the core object that the interfaces with the OpMode
 *              Core components include:  Drive System, Manipulator System, Sensor Network
 */

public class Robot {

//    private ArrayList<DcMotor> driveMotors;
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;

    private ArrayList<DriveWheel> driveWheels;

    private double xPosition;
    private double yPosition;
    private double heading;

    private DriveCommand driveCommand;
    private final double spinScaleFactor = 0.4;

    public Robot() {
        this.xPosition = 0;
        this.yPosition = 0;
        this.heading = 0;
        this.driveCommand = new DriveCommand();
    }

    public DriveCommand getDriveCommand(){
        return driveCommand;
    }

    public DriveWheel getDriveWheel(DriveWheel.WheelPosition wheelPosition){
        DriveWheel driveWheel = null;
        for(DriveWheel dw: driveWheels){
            if(dw.getWheelPosition()==wheelPosition){
                driveWheel = dw;
            }
        }
        return driveWheel;
    }

/*  Old code for explanation of refactor
    public void initDriveMotors(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        driveMotors = new ArrayList<>();
        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);

        for(DcMotor m:driveMotors){
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
*/
    public void initializeStandardDriveWheels(HardwareMap hardwaremap){
        //Initialize the driveWheels array
        driveWheels = new ArrayList<>();

        //Loop through the enumeration in DriveWheel and create a wheel for each position
        for(WheelPosition pos: WheelPosition.values()){
            //Create the drive wheel
            DriveWheel driveWheel = new DriveWheel(pos,hardwaremap);
            //Add it to the array
            driveWheels.add(driveWheel);
        }
    }

    public void setDriveCommand(Gamepad gamepad){
        double forwardDrive = -gamepad.left_stick_y;
        double lateralDrive = gamepad.left_stick_x;

        //  Positive means to spin to the right (clockwise when looking down on robot)
        //  Apply the scale factor to spin
        driveCommand.spin = gamepad.right_stick_x * spinScaleFactor;

        //  drive angle is interpreted as follows:
        //      0 degrees -- forward
        //      90 degrees -- right
        //      180 degrees -- backwards
        //      -90 degrees -- left

        driveCommand.magnitude = Math.hypot(forwardDrive, lateralDrive);
        driveCommand.angleRad = Math.atan2(lateralDrive,forwardDrive);

    }
    public void calculateDrivePowers(){
        //Loop through the drive wheels and set the calculated power

        for(DriveWheel d: driveWheels){
            d.setCalculatedPower(driveCommand);     //Considers translation and spin
        }

        //Now condition the calculated drive powers
        //  --If magnitude of any drive is greater than 1, then scale all down

        double maxCalculatedPowerMagnitude = getMaxCalculatedPowerMagnitude();
        double maxAllowedPower = 1.0;

        //Apply a scale factor if maxAllowedPower is exceeded
        if(maxCalculatedPowerMagnitude>maxAllowedPower){
            double scaleFactor = maxAllowedPower/maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedDrive(scaleFactor);
        }

    }

    private double getMaxCalculatedPowerMagnitude(){
        //Loop through the drive motors and return the max abs value of the calculated drive
        double maxCalculatedPowerMagnitude=0;
        for(DriveWheel dw: driveWheels){
            //get the absolute power from the current drive motor in the loop
            double curMagnitude = Math.abs(dw.getCalculatedPower());  //absolute value
            if(curMagnitude > maxCalculatedPowerMagnitude){
                maxCalculatedPowerMagnitude = curMagnitude;
            }
        }

        return maxCalculatedPowerMagnitude;
    }

    private void applyScaleToCalculatedDrive(double scaleFactor){
        //Loop through each driveWheel and scale calculatedDrive
        for(DriveWheel dw: driveWheels){
            dw.scaleCalculatedPower(scaleFactor);
        }
    }

    public void drive(){
        //Set the calculatedDrive values to the motors
        for(DriveWheel dw: driveWheels){
            dw.setMotorPower();
        }
    }


}
