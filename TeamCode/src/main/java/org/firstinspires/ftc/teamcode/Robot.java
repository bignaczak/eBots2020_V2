package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static java.lang.String.format;
import static org.firstinspires.ftc.teamcode.DriveWheel.*;

/**
 *   CLASS:     Robot
 *   INTENT:    Robot is the core object that interfaces with the OpMode
 *              Core components include:  Drive System, Manipulator System, Sensor Network
 */

public class Robot {
    /***************************************************************
     ******    CLASS VARIABLES
     ***************************************************************/
    // Sensor arrays
    private ArrayList<DriveWheel> driveWheels;
    private ArrayList<EncoderTracker> encoderTrackers = new ArrayList<>();
    private ArrayList<EbotsColorSensor> ebotsColorSensors = new ArrayList<>();
    private ArrayList<EbotsDigitalTouch> ebotsDigitalTouches = new ArrayList<>();
    private ArrayList<EbotsRev2mDistanceSensor> ebotsRev2mDistanceSensors = new ArrayList<>();

    // LED driver
    private RevBlinkinLedDriver revBlinkinLedDriver;
    private ArrayList<EbotsRevBlinkinLedDriver> ebotsRevBlinkinLedDrivers = new ArrayList<>();

    // Camera object detection
    private TFObjectDetector tfod;

    // Field position
    private Pose actualPose;       //Current Pose, which consists of Field Position and Heading
    private Pose targetPose;        //Intended destination of the robot

    //These are robot limitations which should be set using test opmodes to collect data
    private final double topSpeed = 50.0;            //  in / s
    private final double angularTopSpeedDeg = 276.92;;  //  degrees / s

    private ArrayList<SizeCoordinate> robotSizeCoordinates;

    //These are speed settings which are configurable, perhaps should move to a Enumeration
    private final double spinMaxSignal = Speed.FAST.getTurnSpeed();      //Max allowable translate speed in range [0-1];
    private final double translateMaxSignal = Speed.FAST.getMaxSpeed(); //Max allowable translate signal in range [0-1];
    private final double superSlowMoMinTranslateSignal = 0.2;    //Min allowable Super Slow Mo speed
    private final double superSlowMoMinSpinSignal = 0.1;    //Min allowable Super Slow Mo speed

    private Alliance alliance;

    private DriveCommand driveCommand;
    private PoseError poseError;
    private EncoderSetup encoderSetup;
    private EbotsMotionController ebotsMotionController;


    private List<LynxModule> expansionHubs;         //Array list of all expansion hubs on robot
    private BNO055IMU imu;
    private double initialGyroOffset;  //When the robot starts, the gyro is zero in whichever orientation the robot is facing
                                        //  So if robot always faces the center, from the red side, the gyro will read 0 when the
                                        //  robot is actually facing +90°
                                        //  This captures the rotation required to bring the field coordinates frame in line with the
                                        //  the robot coordinate system

    String logTag = "EBOTS";
    boolean debugOn = true;
    /*****************************************************************
     //******    Enumerations
     //****************************************************************/
    public enum RobotSize{
        xSize(CsysDirection.X, 18.0),
        ySize(CsysDirection.Y, 18.0),
        zSize(CsysDirection.Z, 18.0);

        CsysDirection csysDirection;
        double sizeValue;

        RobotSize(CsysDirection csysDirectionIn, double sizeValueIn){
            this.csysDirection = csysDirectionIn;
            this.sizeValue = sizeValueIn;
        }

        public CsysDirection getCsysDirection() {
            return csysDirection;
        }

        public double getSizeValue() {
            return sizeValue;
        }
    }


    /***************************************************************
     ******    CONSTRUCTORS
     ***************************************************************/

    public Robot() {
        this.driveCommand = new DriveCommand();

        //Build the robot physical dimensions
        robotSizeCoordinates = new ArrayList<>();
        for(RobotSize rs: RobotSize.values()){
            robotSizeCoordinates.add(new SizeCoordinate(rs.getCsysDirection(), rs.getSizeValue()));
        }

        //Assumes blue alliance if none stated
        this.alliance = Alliance.BLUE;
        //Assumes a default starting position if none specified
        this.actualPose = new Pose(Pose.PresetPose.INNER_START_LINE, alliance);     //Defaults to INNER and BLUE
        //When no target pose is given, assume Power Shot Launch position
        this.targetPose = new Pose(Pose.PresetPose.LAUNCH_POWER_SHOT, alliance);
        this.poseError = new PoseError(this);

        this.ebotsMotionController = new EbotsMotionController();
        this.encoderSetup = EncoderSetup.TWO_WHEELS;    //Default value if none provided
    }

    public Robot(Pose actualPose){
        this();     //Call the default constructor
        this.actualPose = actualPose;     //Set the input pose
        this.poseError = new PoseError(this);   //recalculate error
    }

    public Robot(Pose pose, Alliance alliance){
        this(pose);     //Set the input pose by calling the above constructor with argument (Pose pose)
        this.alliance = alliance;
    }

    public Robot(AutonParameters autonParameters){
        this();     //chain to constructor with arguments (Pose pose, Alliance alliance)
        this.ebotsMotionController = new EbotsMotionController(autonParameters);
        this.encoderSetup = autonParameters.getEncoderSetup();  //Set the encoderSetup class variable
    }

    public Robot(Pose pose, Alliance alliance, AutonParameters autonParameters){
        this(pose, alliance);     //chain to constructor with arguments (Pose pose, Alliance alliance)
        this.ebotsMotionController = new EbotsMotionController(autonParameters);
        this.encoderSetup = autonParameters.getEncoderSetup();  //Set the encoderSetup class variable
    }

    public Robot(Pose.PresetPose presetPose, Alliance alliance){
        this(new Pose(presetPose,alliance), alliance);  //chained to constructor with arguments (Pose pose, Alliance alliance)
    }

    public Robot(Pose.PresetPose presetPose, Alliance alliance, AutonParameters autonParameters){
        this(new Pose(presetPose,alliance), alliance, autonParameters);  //chained to constructor with arguments (Pose, Alliance, AutonParameters)

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
    public ArrayList<EbotsColorSensor> getEbotsColorSensors(){return this.ebotsColorSensors;}
    public ArrayList<EbotsDigitalTouch> getEbotsDigitalTouches(){return this.ebotsDigitalTouches;}
    public ArrayList<EbotsRev2mDistanceSensor> getEbotsRev2mDistanceSensors(){return this.ebotsRev2mDistanceSensors;}

    public RevBlinkinLedDriver getRevBlinkinLedDriver(){return this.revBlinkinLedDriver;}
    public ArrayList<EbotsRevBlinkinLedDriver> getEbotsRevBlinkinLedDrivers(){return this.ebotsRevBlinkinLedDrivers;}
    public TFObjectDetector getTfod(){return this.tfod;}

    public Alliance getAlliance(){return this.alliance;}
    public Pose getActualPose(){return this.actualPose;}
    public Pose getTargetPose(){return this.targetPose;}
    public PoseError getPoseError(){return this.poseError;}
    public BNO055IMU getImu(){return this.imu;}
    public EncoderSetup getEncoderSetup() {return encoderSetup;}
    public EbotsMotionController getEbotsMotionController(){return this.ebotsMotionController;}

    public double getTopSpeed(){ return this.topSpeed;}
    public double getAngularTopSpeedDeg(){ return this.angularTopSpeedDeg;}
    public double getAngularTopSpeedRad(){ return Math.toRadians(this.angularTopSpeedDeg);}

    public double getSizeCoordinate(CsysDirection dir){
        double sizeValue = 0;
        if(robotSizeCoordinates != null && dir != null && robotSizeCoordinates.size() > 0){
            sizeValue = SizeCoordinate.getSizeFromCoordinates(dir, robotSizeCoordinates);
        }
        return sizeValue;
    }


    public void setTfod(TFObjectDetector tfodIn){
        this.tfod = tfodIn;
    }
    public void setActualPose(Pose pose) {
        this.actualPose = pose;
        //Recalculate error after setting target pose
        this.poseError = new PoseError(this);
    }

    public void setTargetPose(Pose targetPose) {
        this.targetPose = targetPose;
        //Recalculate error after setting target pose
        this.poseError = new PoseError(this);
    }

    public void setDriveCommand(DriveCommand driveCommandIn){
        this.driveCommand = driveCommandIn;
        this.calculateDrivePowers();

    }

    public void setDriveCommand(Gamepad gamepad){
        this.driveCommand = calculateDriveCommandFromGamepad(gamepad);
        this.calculateDrivePowers();
    }

    public void setAlliance(Alliance allianceIn){
        this.alliance = allianceIn;
    }

    /*****************************************************************
     //******    CALCULATED PROPERTIES
     //****************************************************************/

    public boolean isUsingVirtualEncoders(){
        boolean isUsingVirtualEncoders = false;
        for(EncoderTracker e: encoderTrackers){
            if(e.getIsVirtual() == true){
                isUsingVirtualEncoders = true;
                break;
            }
        }
        return  isUsingVirtualEncoders;
    }

    public boolean isStartPositionCorrect(){
        EbotsColorSensor.TapeColor startLineColor = EbotsColorSensor.TapeColor.BLUE;
        RobotSide startLineSide = RobotSide.RIGHT;

        if (this.alliance == Alliance.RED) {
            startLineColor = EbotsColorSensor.TapeColor.RED;
            startLineSide = RobotSide.LEFT;
        }

        //Update whether the start position has been achieved
        return EbotsColorSensor.isSideOnColor(this.getEbotsColorSensors(), startLineSide, startLineColor);
    }


    /*****************************************************************
     //******    CLASS INSTANCE METHODS
     //****************************************************************/
    public void updateStartPose(StartLine.LinePosition startLinePosition){
        Pose startingPose = calculateStartingPose(startLinePosition);     //robot object exists at this point
        this.setActualPose(startingPose);
    }

    private Pose calculateStartingPose(StartLine.LinePosition startLinePosition){
        //Starting poses are handled in an enumeration within Pose

        Pose.PresetPose presetPose;
        if(startLinePosition == StartLine.LinePosition.INNER){
            presetPose = Pose.PresetPose.INNER_START_LINE;
        } else{
            presetPose = Pose.PresetPose.OUTER_START_LINE;
        }

        Pose startingPose = new Pose(presetPose, this.getAlliance());
        return startingPose;
    }

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

    public void initializeEncoderTrackers(AutonParameters autonParameters){
        //boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering Robot.initializeEncoderTrackers(AutonParameters)...");

        boolean isVirtual = autonParameters.usesSimulatedEncoders();
        initializeEncoderTrackers(autonParameters.getEncoderSetup(), isVirtual);

        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("Number of encoders initialized: ");
            sb.append(this.encoderTrackers.size());
            for(EncoderTracker e: this.encoderTrackers){
                sb.append("\n");
                sb.append(e.toString());
            }
            Log.d(logTag, sb.toString());
        }
    }

    public void initializeEncoderTrackers(EncoderSetup encoderSetup, boolean isVirtual){
        //Initializes encoder trackers and maps them to wheelPosition motors
        this.encoderSetup = encoderSetup;   //Capture the encoder setup in robot member variable

        if(isVirtual) {
            initializeVirtualEncoderTrackers();
        } else{
            EncoderModel encoderModel = encoderSetup.getEncoderModel();

            //DcMotorEx motor, RobotOrientation robotOrientation, EncoderModel encoderModel
            final DcMotorEx forwardEncoderMotor = this.getDriveWheel(WheelPosition.BACK_RIGHT).getWheelMotor();
            final DcMotorEx lateralEncoderMotor = this.getDriveWheel(WheelPosition.FRONT_RIGHT).getWheelMotor();
            encoderTrackers.add(new EncoderTracker(forwardEncoderMotor, RobotOrientation.FORWARD, encoderModel));
            encoderTrackers.add(new EncoderTracker(lateralEncoderMotor, RobotOrientation.LATERAL, encoderModel));
            if (encoderSetup == EncoderSetup.THREE_WHEELS) {
                //Create a second forward encoder
                final DcMotorEx forward2EncoderMotor = this.getDriveWheel(WheelPosition.FRONT_LEFT).getWheelMotor();
                EncoderTracker thirdEncoder = new EncoderTracker(forward2EncoderMotor, RobotOrientation.FORWARD, encoderModel);
                thirdEncoder.setSpinBehavior(EncoderTracker.SpinBehavior.DECREASES_WITH_ANGLE);
                encoderTrackers.add(thirdEncoder);
            }
        }
    }

    private void initializeVirtualEncoderTrackers() {
        //Initializes virtual encoders
        encoderTrackers.add(new EncoderTracker(true, RobotOrientation.FORWARD));
        encoderTrackers.add(new EncoderTracker(true, RobotOrientation.LATERAL));

        if (encoderSetup == EncoderSetup.THREE_WHEELS) {
            //Create a second forward encoder
            EncoderTracker thirdEncoder = new EncoderTracker(true, RobotOrientation.FORWARD);
            thirdEncoder.setSpinBehavior(EncoderTracker.SpinBehavior.DECREASES_WITH_ANGLE);
            encoderTrackers.add(thirdEncoder);
        }
    }

        public void initializeColorSensors(HardwareMap hardwareMap){
        //Create color sensors used on the robot

        //Make sure the list is empty before initializing
        if(ebotsColorSensors.size() > 0) ebotsColorSensors.clear();

        //Add four color sensors are located at each wheel location, based on SensorLocation enum
        for(EbotsColorSensor.SensorLocation loc: EbotsColorSensor.SensorLocation.values()){
            ebotsColorSensors.add(new EbotsColorSensor(loc, hardwareMap));
        }
    }

    public void initializeEbotsDigitalTouches(HardwareMap hardwareMap){
        //Create digitalTouch sensors used on the robot

        //Make sure the list is empty before initializing
        if(ebotsDigitalTouches.size() > 0) ebotsColorSensors.clear();

        //Add EbotsDigitalTouch sensor for each ButtonFunction enum value
        for(EbotsDigitalTouch.ButtonFunction buttonFunction: EbotsDigitalTouch.ButtonFunction.values()){
            ebotsDigitalTouches.add(new EbotsDigitalTouch(buttonFunction,hardwareMap));
        }
    }

    public void initializeRevBlinkinLedDriver(HardwareMap hardwareMap){
        //Initialize the LED lights
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    }

    public void initializeEbotsRev2mDistanceSensors(HardwareMap hardwareMap){
        for(RobotSide rs: RobotSide.values()){
            ebotsRev2mDistanceSensors.add(new EbotsRev2mDistanceSensor(rs, hardwareMap));
        }
    }

    public void initializeEbotsRevBlinkinDriver(HardwareMap hardwareMap){
        for(EbotsRevBlinkinLedDriver.LedLocation ledLocation: EbotsRevBlinkinLedDriver.LedLocation.values()){
            ebotsRevBlinkinLedDrivers.add(new EbotsRevBlinkinLedDriver(ledLocation, this.alliance, hardwareMap));
        }
    }

    public void initializeExpansionHubsForBulkRead(HardwareMap hardwareMap) {
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering initializeExpansionHubsForBulkRead...");


        // Generate a list of all the robot's Expansion/Control Hubs
        expansionHubs = hardwareMap.getAll(LynxModule.class);

        // Set all Expansion hubs to use the AUTO Bulk Caching mode.
        // This setting performs one bulk read and allows each sensor to queried once
        // If a second query occurs, then additional bulk reads will occur
        for (LynxModule module : expansionHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void bulkReadSensorInputs(long loopDuration, boolean includeColorSensors, boolean includeDistanceSensors){
        //This should be done once per control loop
        //It interfaces with the REV Expansion hubs to read all the values stored in its cache
        //These must be moved to variables for further accessing.
        //Duplicating calls to the hardware will cause additional bulk reads if in AUTO mode, slowing control loop
        //Look in examples ConceptMotorBulkRead for further guidance
        //boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering Robot.bulkReadSensorInputs...");

        //if using virtual encoders, simulate the loop output
        if(this.isUsingVirtualEncoders()){
            for(EncoderTracker e: this.encoderTrackers){
                if(debugOn) Log.d(logTag, "Sending to simulateLoopOutput: " + e.toString());
                e.simulateLoopOutput(this, loopDuration);
                if(debugOn){
                    StringBuilder sb = new StringBuilder();
                    sb.append("Back in Robot.BulkReadSensorInputs\n");
                    sb.append(e.toString());
                    Log.d(logTag, sb.toString());
                }
            }
        }else {
            //Read in the Encoder Values
            for (EncoderTracker e : encoderTrackers) {
                e.setNewReading();
            }
        }


        boolean readImu = (this.getEncoderSetup() == EncoderSetup.TWO_WHEELS);
        //If the Imu is being used, read it
        if(readImu) {
            //Set the newHeadingReadingDeg variable for the pose
            if(!isUsingVirtualEncoders()){
                // Use the imu if not using virtual encoders
                float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                this.setNewHeadingReadingDegFromGyro(gyroReading);
            } else{
                // Set newHeadingReadingDeg based on best estimate
                double estimatedNewHeading = this.actualPose.getHeadingDeg() + estimateHeadingChangeDeg(loopDuration);
                this.actualPose.setNewHeadingReadingDeg(estimatedNewHeading);
            }
        }

        //Read the digitalTouches
        for(EbotsDigitalTouch edt: ebotsDigitalTouches){
            edt.setIsPressed();
        }


        //Read the colorSensors
        if(includeColorSensors) {
            for (EbotsColorSensor ecs : ebotsColorSensors) {
                ecs.setColorValue();
            }
        }

        if(includeDistanceSensors) {
            //Read the distance Sensors
            for (EbotsRev2mDistanceSensor ds : ebotsRev2mDistanceSensors) {
                ds.setDistanceInches();
            }
        }
    }

    public void bulkReadSensorInputs(long loopDuration, int loopCount, StopWatch stopWatch){
        //OVERLOAD FOR PERFORMANCE TUNING

        //This should be done once per control loop
        //It interfaces with the REV Expansion hubs to read all the values stored in its cache
        //These must be moved to variables for further accessing.
        //Duplicating calls to the hardware will cause additional bulk reads if in AUTO mode, slowing control loop
        //Look in examples ConceptMotorBulkRead for further guidance
        //boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering Robot.bulkReadSensorInputs...");

        //Debug parameters
        long splitTimeMillis = stopWatch.getElapsedTimeMillis();
        String operation = "";

        if (debugOn)operation = "ReadEncoders";
        //if using virtual encoders, simulate the loop output
        if(this.isUsingVirtualEncoders()){
            for(EncoderTracker e: this.encoderTrackers){
                //EncoderTracker e = this.encoderTrackers.get(i);
                if(debugOn) Log.d(logTag, "Sending to simulateLoopOutput: " + e.toString());
                e.simulateLoopOutput(this, loopDuration);
                if(debugOn){
                    StringBuilder sb = new StringBuilder();
                    sb.append("Back in Robot.BulkReadSensorInputs\n");
                    sb.append(e.toString());
                    Log.d(logTag, sb.toString());
                }
            }
        }else {
            //Read in the Encoder Values
            for (EncoderTracker e : encoderTrackers) {
                e.setNewReading();
            }
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);



        if (debugOn) operation = "Read IMU";
        boolean readImu = (this.getEncoderSetup() == EncoderSetup.TWO_WHEELS);
        //If the Imu is being used, read it
        if(readImu) {
            //Set the newHeadingReadingDeg variable for the pose
            if(!isUsingVirtualEncoders()){
                // Use the imu if not using virtual encoders
                float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                this.setNewHeadingReadingDegFromGyro(gyroReading);
            } else{
                // Set newHeadingReadingDeg based on best estimate
                double estimatedNewHeading = this.actualPose.getHeadingDeg() + estimateHeadingChangeDeg(loopDuration);
                this.actualPose.setNewHeadingReadingDeg(estimatedNewHeading);
            }
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);



        //Read the colorSensors
        if(debugOn) operation = "Read colorSensors";
        for(EbotsColorSensor ecs: ebotsColorSensors){
            ecs.setColorValue();
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);


        if(debugOn) operation = "Read digitalTouches";
        //Read the digitalTouches
        for(EbotsDigitalTouch edt: ebotsDigitalTouches){
            edt.setIsPressed();
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);


        //Read the distance Sensors
        if(debugOn) operation = "Read distanceSensors";
        for(EbotsRev2mDistanceSensor ds: ebotsRev2mDistanceSensors){
            ds.setDistanceInches();
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);

    }


    public void logEncoderTrackers(){
        //boolean debugOn = false;
        StringBuilder sb = new StringBuilder();
        sb.append("Entering Robot.logEncoderTrackers...");
        for(EncoderTracker e: this.encoderTrackers){
            sb.append("\n");
            sb.append(e.toString());
        }
        if(debugOn){
            Log.d(logTag,sb.toString());
        }
    }

    public void testMotors(LinearOpMode opMode, Gamepad gamepad1){
        StopWatch stopWatch = new StopWatch();
        long lockoutTimeMillis = 750L;

        this.driveCommand = new DriveCommand();
        this.driveCommand.setMagnitude(0.2);
        this.calculateDrivePowers();

        for (DriveWheel dw: driveWheels){
            this.stop();
            dw.setCalculatedPower(driveCommand);
            dw.setMotorPower();
            while(!opMode.isStopRequested()
                    && !opMode.isStarted()
                    && !(gamepad1.x && stopWatch.getElapsedTimeMillis()>lockoutTimeMillis)
                    ) {
                opMode.telemetry.clearAll();
                opMode.telemetry.addData("Current Motor", dw.getWheelPosition().toString());
                opMode.telemetry.addLine("Push X to go to next motor");
                opMode.telemetry.update();
            }
            stopWatch.reset();
        }

        this.stop();
        this.driveCommand.setMagnitude(0);

    }

    public double estimateHeadingChangeDeg(long timeStepMillis){
        //Returns the estimated change in the robots heading based on the driveCommand and speed attributes of the robot
        //This is used when simulated loop output and the gyro is not available
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering estimateHeadingChangeDeg...");

        double spinSignal = this.driveCommand.getSpin();
        double angularTopSpeedDeg = this.angularTopSpeedDeg;
        double actualAngularSpeedDeg = spinSignal * angularTopSpeedDeg;

        double rotationAngleDeg = actualAngularSpeedDeg * (timeStepMillis / 1000.0);
        return rotationAngleDeg;
    }

    public DriveCommand calculateDriveCommandFromGamepad(Gamepad gamepad){
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
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering calculateDriveCommandFromGamepad...");


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
        DriveCommand driveCommand = new DriveCommand();
        driveCommand.setMagnitudeAndDriveAngle(forwardInput, lateralInput, this.translateMaxSignal);
        driveCommand.setSpinDrive(spinInput, this.spinMaxSignal);
        return driveCommand;
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
        //Also update the DriveCommand since it must be scaled back for virtual encoders
        driveCommand.setMagnitude(driveCommand.getMagnitude() * scaleFactor);
        driveCommand.setSpin(driveCommand.getSpin() * scaleFactor);
    }

    public void drive(){
        //boolean debugOn = false;
        if(debugOn) {
            Log.d(logTag, "Entering robot.drive()...");
            Log.d(logTag, "with " + this.driveCommand.toString());
        }
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

    public void updateActualPose(){
        //Intended to accept a PoseChange object and update the robot's pose accordingly

        //boolean debugOn = false;
        if(debugOn) Log.d(logTag,"Entering updateActualPose...");
        // Calculate move since last loop
        PoseChange poseChange = new PoseChange(this);

        //First update heading
        actualPose.setHeadingDeg(poseChange.getSpinAngleDeg() + poseChange.getSpinAngleDeg());
        actualPose.setX(actualPose.getX() + poseChange.getIncrementalFieldMovement().getxPosition());
        actualPose.setY(actualPose.getY() + poseChange.getIncrementalFieldMovement().getyPosition());
    }


    public void updateAllSensorValues(){
        // During auton, changes in robot position are calculated based on comparing new readings to sensor values
        // After these calculations have been performed, all new readings must be transferred to the sensor values
        // This method goes through each sensor type and performs the value transfer

        //boolean debugOn = false;
        if(debugOn) Log.d(logTag,"Entering updateAllSensorValues...");


        //Transfer encoder readings to current clicks
        for(EncoderTracker e:encoderTrackers){
            e.updateEncoderCurrentClicks();
        }

        //Transfer newHeadingReadingDeg to headingDeg for actual pose
        this.getActualPose().updateHeadingWithReading();
    }

    public RevBlinkinLedDriver.BlinkinPattern updateLedPattern(){
        RevBlinkinLedDriver.BlinkinPattern pattern;
        if (alliance == Alliance.BLUE) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
        }
        this.getRevBlinkinLedDriver().setPattern(pattern);
        return pattern;
    }

    public void toggleAlliance(){
        if (alliance == Alliance.RED) {
            alliance = Alliance.BLUE;
        } else {
            alliance = Alliance.RED;
        }
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

    public void logSensorData(String logTag){
        Log.d(logTag, "Actual " + this.actualPose.toString());
        Log.d(logTag, EbotsRev2mDistanceSensor.printAll(this.getEbotsRev2mDistanceSensors()));
        Log.d(logTag, EbotsColorSensor.printColorsObserved(this.getEbotsColorSensors()));
        Log.d(logTag, EncoderTracker.printAll(this.getEncoderTrackers()));

        for(EbotsColorSensor ecs: ebotsColorSensors) {
            Log.d(logTag, ecs.toString());
        }
    }

    @Override
    public String toString(){
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag,"Entering robot.ToString...");

        StringBuilder outString = new StringBuilder();
        if(actualPose != null) outString.append("Actual " + actualPose.toString());
        if(targetPose != null) outString.append("\n" + "Target " + targetPose.toString());
        return outString.toString();
    }
}
