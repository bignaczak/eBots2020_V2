package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class StatePrematchSetup implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    TFObjectDetector tfod;


    boolean isSetupCorrect;
    boolean wasSetupCorrect = false;
    StopWatch setupStopWatch = new StopWatch();
    RobotSide robotSide;
    EbotsColorSensor.TapeColor tapeColor;
    RevBlinkinLedDriver.BlinkinPattern alliancePattern;
    double nominalDistance;
    double distanceTolerance = 10;
    double actualDistance;

    final boolean debugOn = true;
    final String logTag = "EBOTS";

    // ***********   CONSTRUCTOR   ***********************
    public StatePrematchSetup(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.PREMATCH_SETUP;
        this.nextAutonStateEnum = AutonStateEnum.DETECT_STARTER_STACK;
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
        long stableSetupTimeTarget = 2000L;
        boolean isCorrectSetupStable = setupStopWatch.getElapsedTimeMillis() > stableSetupTimeTarget;

        // Verify that the setup is stable:
        //  a) that it is correct AND
        //  b) has been correct for some period of time (stableSetupTimeTarget)
        boolean isSetupStable = (isSetupCorrect & isCorrectSetupStable);

        // Exit if either:
        //  a) setup is stable OR
        //  b) opMode is started
        boolean verdict = (isSetupStable | this.opMode.isStarted());
        return verdict;
    }

    @Override
    public void performStateSpecificTransitionActions() {

    }

    @Override
    public void performStateActions() {
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": entering performStateActions");
        // Update the readings from the sensors
        performSensorHardwareReads();
        // Determine robotSide, tapeColor, and alliancePattern
        assignMeasurementParameters();

        // Now check if three conditions are met
        //  1) Touching the back wall
        //  2) Positioned over the startline tape
        //  3) On the correct startline
        boolean isOnWall = isTouchingBackWall();
        boolean isOnTape = isCorrectRobotSideOnCorrectColorTape();
        boolean isCorrectStartLine = isRobotPlacedOnCorrectStartLine();

        // Provide the verdict for setup correctness for this pass through loop
        isSetupCorrect = isOnTape && isOnWall && isCorrectStartLine;

        // If the setup just turned from incorrect to correct, reset the setupStopWatch
        // Exit conditions require the setup to be stable for some period of time
        // which is measured with the setupStopWawtch
        if (isSetupCorrect & !wasSetupCorrect){
            setupStopWatch.reset();
        }

        // Use this to pass to the next loop iteration
        wasSetupCorrect = isSetupCorrect;

        updateTelemetry();

        updateLedDisplay();

    }

    // ***********   CLASS MEMBER METHODS   ***********************
    private void updateLedDisplay() {
        //display LED lights, green is good to go, red means there is a problem in setup
        RevBlinkinLedDriver.BlinkinPattern pattern = (isSetupCorrect) ? RevBlinkinLedDriver.BlinkinPattern.GREEN : alliancePattern;
        robot.getRevBlinkinLedDriver().setPattern(pattern);
    }

    private boolean isTouchingBackWall() {
        EbotsDigitalTouch backWall = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.DETECT_BACK_WALL,
                robot.getEbotsDigitalTouches());
        return backWall.getIsPressed();
    }

    private boolean isCorrectRobotSideOnCorrectColorTape() {
        return EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), robotSide, tapeColor);
    }

    private boolean isRobotPlacedOnCorrectStartLine() {
        boolean isCorrectStartLine = false;

        //calculate expected distance from the wall the distance sensor to the wall
        double playFieldHalfWidth = new PlayField().getFieldWidth() /2;

        // this is refactored to use the robot's currentPose, which was set based on the startLine
        // double nominalDistance = playFieldHalfWidth - Math.abs(startLineY) - robotWidth;
        nominalDistance = playFieldHalfWidth - Math.abs(robot.getActualPose().getY()) -
                (robot.getSizeCoordinate(CsysDirection.Y) / 2);

        // set which distance sensor will provide the reading
        RobotSide distanceSide = (robot.getAlliance() == Alliance.RED) ? RobotSide.RIGHT : RobotSide.LEFT;
        actualDistance = EbotsRev2mDistanceSensor.getDistanceForRobotSide(distanceSide, robot.getEbotsRev2mDistanceSensors());

        //check distance for robot side
        if (actualDistance >= (nominalDistance - distanceTolerance) && actualDistance <= (nominalDistance + distanceTolerance)){
            isCorrectStartLine = true;
        }

        return isCorrectStartLine;
    }


    private void assignMeasurementParameters() {
        // Check alliance
        if (robot.getAlliance() == Alliance.RED) {
            robotSide = RobotSide.LEFT;
            tapeColor = EbotsColorSensor.TapeColor.RED;
            alliancePattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else {
            robotSide = RobotSide.RIGHT;
            tapeColor = EbotsColorSensor.TapeColor.BLUE;
            alliancePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }
    }


    private void performSensorHardwareReads() {
        //Read the values for the color sensors from hardware into variables
        for (EbotsColorSensor sensor : robot.getEbotsColorSensors()) {
            sensor.setColorValue();
        }

        //Read the values for the distance sensors from hardware into variables
        for(EbotsRev2mDistanceSensor distanceSensor: robot.getEbotsRev2mDistanceSensors()){
            distanceSensor.setDistanceInches();
        }

        //Read the values for the digital touch sensors
        for(EbotsDigitalTouch ebotsDigitalTouch: robot.getEbotsDigitalTouches()){
            ebotsDigitalTouch.setIsPressed();
        }
    }

    // Initialize the tensorflow object detection
    private void initTfod() {
        //Class variables for using camera
        String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
        String LABEL_FIRST_ELEMENT = "Quad";
        String LABEL_SECOND_ELEMENT = "Single";
        String VUFORIA_KEY =
                "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";
        VuforiaLocalizer vuforia = null;


        int tfodMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        // Set the robot variable on the robot
        this.robot.setTfod(tfod);
    }


    private void updateTelemetry(){
        this.opMode.telemetry.addLine("Current autonState: " + this.currentAutonStateEnum.toString());
        this.opMode.telemetry.addLine("opMode is Started / Active: " + opMode.isStarted() + "/" + opMode.opModeIsActive());
        this.opMode.telemetry.addLine("Setup Config: " + robot.getAlliance() + " | " + robot.getActualPose().toString());
        this.opMode.telemetry.addLine("Actual -- Expected[<-->]: " +
                String.format("%.2f", (this.actualDistance)) + " -- " +
                String.format("%.2f", (nominalDistance - distanceTolerance)) + "<-->" +
                ", " + String.format("%.2f", (nominalDistance + distanceTolerance)));
        this.opMode.telemetry.addLine("Robot is on the back wall: " + isTouchingBackWall());
        this.opMode.telemetry.addLine("Robot is on the correct tape: " + isCorrectRobotSideOnCorrectColorTape());
        this.opMode.telemetry.addLine("Robot is on the correct start line: " + isRobotPlacedOnCorrectStartLine());
        this.opMode.telemetry.addLine("Overall correct set up: " + isSetupCorrect + " - " + setupStopWatch.toString());
        this.opMode.telemetry.addLine("LED pattern: " + robot.getRevBlinkinLedDriver().toString());
        this.opMode.telemetry.update();

    }
}
