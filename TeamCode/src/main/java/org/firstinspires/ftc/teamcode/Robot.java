package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.List;

import static java.lang.String.format;
import static org.firstinspires.ftc.teamcode.DriveWheel.*;

/**
 *   CLASS:     Robot
 *   INTENT:    Robot is the core object that the interfaces with the OpMode
 *              Core components include:  Drive System, Manipulator System, Sensor Network
 */

public class Robot {
    /***************************************************************
     ******    CLASS VARIABLES
     ***************************************************************/

    private ArrayList<DriveWheel> driveWheels;
    private ArrayList<EncoderTracker> encoderTrackers;

    private Pose actualPose;       //Current Pose, which consists of Field Position and Heading
    private Pose targetPose;        //Intended destination of the robot

    //These are robot limitations which should be set using test opmodes to collect data
    private final double topSpeed = 50.0;            //  in / s
    private final double angularTopSpeedDeg = 276.92;;  //  degrees / s

    //These are speed settings which are configurable, perhaps should move to a Enumeration
    private final double spinMaxSignal = 0.4;      //Max allowable translate speed in range [0-1];
    private final double translateMaxSignal = 1.0; //Max allowable translate signal in range [0-1];
    private final double superSlowMoMinSignal = 0.2;    //Min allowable Super Slow Mo speed

    private DriveCommand driveCommand;
    private PoseError poseError;
    private EncoderSetup encoderSetup;


    private List<LynxModule> expansionHubs;         //Array list of all expansion hubs on robot
    private BNO055IMU imu;
    private double initialGyroOffset;  //When the robot starts, the gyro is zero in whichever orientation the robot is facing
                                        //  So if robot always faces the center, from the red side, the gyro will read 0 when the
                                        //  robot is actually facing +90°
                                        //  This captures the rotation required to bring the field coordinates frame in line with the
                                        //  the robot coordinate system


    /***************************************************************
     ******    CONSTRUCTORS
     ***************************************************************/

    public Robot() {
        this.driveCommand = new DriveCommand();

        //Assumes a default starting position if none specified
        this.actualPose = new Pose(Pose.PresetPose.INNER_START_LINE, Alliance.BLUE);     //Defaults to INNER and BLUE
        //When no target pose is given, assume Power Shot Launch position
        this.targetPose = new Pose(Pose.PresetPose.LAUNCH_POWER_SHOT, Alliance.BLUE);
        this.poseError = new PoseError(this);
    }

    public Robot(Pose.PresetPose presetPose, Alliance alliance){
        this();
        this.actualPose = new Pose(presetPose,alliance);      //Rewrite the default value using the passed parameters
        this.poseError = new PoseError(this);       //Recalculate pose error after setting actual position
    }


    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/
    public DriveCommand getDriveCommand(){
        return driveCommand;
    }

    public DriveWheel getDriveWheel(DriveWheel.WheelPosition wheelPosition){
        DriveWheel driveWheel = null;
        for(DriveWheel dw: driveWheels){
            if(dw.getWheelPosition()==wheelPosition){
                driveWheel = dw;
                break;
            }
        }
        return driveWheel;
    }

    public ArrayList<DriveWheel> getDriveWheels(){
        return driveWheels;
    }
    public ArrayList<EncoderTracker> getEncoderTrackers(){return encoderTrackers;}

    public Pose getActualPose(){return this.actualPose;}
    public Pose getTargetPose(){return this.targetPose;}
    public PoseError getPoseError(){return this.poseError;}
    public BNO055IMU getImu(){return this.imu;}
    public EncoderSetup getEncoderSetup() {return encoderSetup;}

    public double getTopSpeed(){ return this.topSpeed;}
    public double getAngularTopSpeedDeg(){ return this.angularTopSpeedDeg;}
    public double getAngularTopSpeedRad(){ return Math.toRadians(this.angularTopSpeedDeg);}


    /*****************************************************************
     //******    CLASS INSTANCE METHODS
     //****************************************************************/

    public void setInitialGyroOffset(double gyroReading){
        //  This is run right after creating the robot during initialization
        //  This captures the rotation required to bring the field coordinates frame in line with the
        //  the robot coordinate system
        initialGyroOffset = this.actualPose.getHeadingDeg() - gyroReading;
    }

    public void initializeImu(HardwareMap hardwareMap){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        this.setInitialGyroOffset(this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

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

    public void initializeEncoderTrackers(EncoderSetup encoderSetup, boolean isVirtual){
        this.encoderSetup = encoderSetup;   //Capture the encoder setup in robot member variable

        if(isVirtual) {
            encoderTrackers.add(new EncoderTracker(isVirtual, RobotOrientation.FORWARD));
            encoderTrackers.add(new EncoderTracker(isVirtual, RobotOrientation.LATERAL));

            if (encoderSetup == EncoderSetup.THREE_WHEELS) {
                //Create a second forward encoder
                encoderTrackers.add(new EncoderTracker(isVirtual, RobotOrientation.FORWARD));
            }
        } else{
            //DcMotorEx motor, RobotOrientation robotOrientation, EncoderModel encoderModel
            final DcMotorEx forwardEncoderMotor = this.getDriveWheel(WheelPosition.FRONT_LEFT).getWheelMotor();
            final DcMotorEx lateralEncoderMotor = this.getDriveWheel(WheelPosition.FRONT_RIGHT).getWheelMotor();
            encoderTrackers.add(new EncoderTracker(forwardEncoderMotor, RobotOrientation.FORWARD, EncoderModel.REV));
            encoderTrackers.add(new EncoderTracker(lateralEncoderMotor, RobotOrientation.LATERAL, EncoderModel.REV));
            if (encoderSetup == EncoderSetup.THREE_WHEELS) {
                //Create a second forward encoder

                final DcMotorEx forward2EncoderMotor = this.getDriveWheel(WheelPosition.BACK_LEFT).getWheelMotor();
                encoderTrackers.add(new EncoderTracker(forward2EncoderMotor, RobotOrientation.FORWARD, EncoderModel.REV));
            }
        }
    }

    public void bulkReadSensorInputs(boolean readImu){
        //This should be done once per control loop
        //It interfaces with the REV Expansion hubs to read all the values stored in its cache
        //These must be moved to variables for further accessing.
        //Duplicating calls to the hardware will cause additional bulk reads if in AUTO mode, slowing control loop
        //Look in examples ConceptMotorBulkRead for further guidance

        //Read in the Encoder Values
        for(EncoderTracker e: encoderTrackers){
            e.setNewReading();
        }

        //If the Imu is being used, read it
        if(readImu) {
            //Read in the imu
            float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            this.setNewHeadingReadingDegFromGyro(gyroReading);
        }
    }

    public void initializeExpansionHubsForBulkRead(HardwareMap hardwareMap) {
        // Generate a list of all the robot's Expansion/Control Hubs
        expansionHubs = hardwareMap.getAll(LynxModule.class);

        // Set all Expansion hubs to use the AUTO Bulk Caching mode.
        // This setting performs one bulk read and allows each sensor to queried once
        // If a second query occurs, then additional bulk reads will occur
        for (LynxModule module : expansionHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    public void setDriveCommand(Gamepad gamepad){
        //  Robot Drive Angle is interpreted as follows:
        //
        //      0 degrees -- forward - (Positive X-Direction)
        //      90 degrees -- left   - (Positive Y-Direction)
        //      180 degrees -- backwards (Negative X-Direction)
        //      -90 degrees -- right    (Negative Y-Direction)
        //
        //  NOTE: This convention follows the right hand rule method, where :
        //      +X --> Forward, +Y is Left, +Z is up
        //   +Spin --> Counter clockwise

        //Read in the gamepad inputs
        double forwardInput = -gamepad.left_stick_y;  //reversing sign because up on gamepad is negative
        double lateralInput = -gamepad.left_stick_x;  //reversing sign because right on gamepad is positive
        double spinInput = -gamepad.right_stick_x;    //Positive means to spin to the left (counterclockwise (CCW) when looking down on robot)

        //todo:  Add the super Slow Mo controls
        //Get the input for Super Slo-Mo
//        double superSloMoInput = 1-gamepad.left_trigger;
//        if(superSloMoInput < superSlowMoMinSignal){
//            superSloMoInput = superSlowMoMinSignal;
//        }
//        translateMaxSignal = superSloMoInput;

        //Set the values for the robot's driveCommand object
        driveCommand.setMagnitudeAndDriveAngle(forwardInput, lateralInput, translateMaxSignal);
        driveCommand.setSpinDrive(spinInput, spinMaxSignal);

    }

    public void calculateDrivePowers(){
        //Loop through the drive wheels and set the calculated power

        for(DriveWheel dw: driveWheels){
            dw.setCalculatedPower(driveCommand);     //Considers translation and spin
        }

        //Now condition the calculated drive powers
        //  --If magnitude of any drive is greater than 1, then scale all down
        //    Note:  this scaling is different from scaling due to Speed objects
        //           this makes sures the motors aren't over-driven, which would cause erratic controls
        //           For instance, it is possible to have both translate and spin speeds governed to 0.5
        //           but still drive a motor to its full potential of 1.0

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

    public void applyScaleToCalculatedDrive(double scaleFactor){
        //Loop through each driveWheel and scale calculatedDrive
        for(DriveWheel dw: driveWheels){
            dw.scaleCalculatedPower(scaleFactor);
        }
        //todo:  Is this necessary?  Maybe for the virtual encoders
        //Also update the DriveCommand since it must be scaled back
        driveCommand.setMagnitude(driveCommand.getMagnitude() * scaleFactor);
        driveCommand.setSpin(driveCommand.getSpin() * scaleFactor);
    }

    public void drive(){
        //Set the calculatedDrive values to the motors
        for(DriveWheel dw: driveWheels){
            dw.setMotorPower();
        }
    }

    public void stop(){
        //Set the calculatedDrive values to the motors
        for(DriveWheel dw: driveWheels){
            dw.stopMotor();
        }
    }

    public void updateActualPose(PoseChange poseChange){
        //Intended to accept a PoseChange object and update the robot's pose accordingly
        //First update heading
        actualPose.setHeadingDeg(poseChange.getSpinAngleDeg() + poseChange.getSpinAngleDeg());
        actualPose.setX(actualPose.getX() + poseChange.getIncrementalFieldMovement().getxPosition());
        actualPose.setY(actualPose.getY() + poseChange.getIncrementalFieldMovement().getyPosition());
    }


    public void updateAllSensorValues(){
        // During auton, changes in robot position are calculated based on comparing new readings to sensor values
        // After these calculations have been performed, all new readings must be transferred to the sensor values
        // This method goes through each sensor type and performs the value transfer

        //Transfer encoder readings to current clicks
        for(EncoderTracker e:encoderTrackers){
            e.updateEncoderCurrentClicks();
        }

        //Transfer newHeadingReadingDeg to headingDeg for actual pose
        this.getActualPose().updateHeadingWithReading();
    }

    public void setHeadingFromGyro(double gyroHeading){
        double fieldHeading = calculateHeadingFromGyro(gyroHeading);
        this.actualPose.setHeadingDeg(fieldHeading);
    }

    public void setNewHeadingReadingDegFromGyro(double gyroHeading){
        double fieldHeading = calculateHeadingFromGyro(gyroHeading);
        this.actualPose.setNewHeadingReadingDeg(fieldHeading);
    }

    private double calculateHeadingFromGyro(double gyroHeading){
        return gyroHeading + this.initialGyroOffset;
    }

    @Override
    public String toString(){
        StringBuilder outString = new StringBuilder();
        outString.append("Robot Position: " + actualPose.toString());
        outString.append("\n");
        outString.append("Encoder Setup: " + encoderSetup.toString());
        return outString.toString();
    }
}
