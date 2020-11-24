package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.String.format;

public class EbotsMotionController {
    /**
     * This class functions as a PID controller for translation and rotation
     * using a 3 encoder setup
     */

    /*****************************************************************
    //******    CLASS VARIABLES
    //***************************************************************/
    private Speed speed;
    private GyroSetting gyroSetting;
    private Accuracy accuracy;
    private SoftStart softStart;

    /*****************************************************************
    //******    ENUMERATIONS
    //***************************************************************/


    /*****************************************************************
    //******    CONSTRUCTORS
    //***************************************************************/
    public EbotsMotionController (){
        this.speed = Speed.FAST;
        this.gyroSetting = GyroSetting.NONE;
        this.accuracy = Accuracy.STANDARD;
        this.softStart = SoftStart.MEDIUM;
    }

    public EbotsMotionController(AutonParameters autonParameters){
        this.speed = autonParameters.getSpeed();
        this.gyroSetting = autonParameters.getGyroSetting();
        this.accuracy = autonParameters.getAccuracy();
        this.softStart = autonParameters.getSoftStart();
    }

    /*****************************************************************
     //******    CLASS INSTANCE METHODS
     //***************************************************************/

    public void moveToTargetPose(Robot robot, LinearOpMode opMode){
        /*
        1) Calculate PoseError -- x,y, heading components of error and errorSums for integrator (using field coordinate system)
        2) Compute the DriveCommand for the robot considering error & speed(in the robot's coordinate system)
        3) Calculate the motor powers based on DriveCommand
        4) Scale drive vectors if softStart is in effect
        5) Apply the calculated motor powers
        6) Continue looping until exit conditions occur
         */
        //todo:  Add a Defend Position option

        final boolean debugOn = true;
        final String logTag = "EBOTS";

        //Log position at start of travel leg
        if (debugOn) {
            Log.d(logTag, "Entering moveToTargetPose...");
            Log.d(logTag, "Start Position " + robot.getActualPose().toString());
            Log.d(logTag, "Target Position " + robot.getTargetPose().toString());
            Log.d(logTag, "Error " + robot.getPoseError().toString());
            Log.d(logTag, "Speed Settings " + speed.toString());
            Log.d(logTag, "Accuracy Settings " + accuracy.toString());
        }

        //Zero out integrator errorSums for X, Y, and heading
        robot.getPoseError().resetErrorSums();
        //Set the variable for whether the imu will be read for heading, which is true when using TWO_WHEELS
        //todo:  Add consideration for updating THREE_WHEEL calculated heading with actual reading
        boolean readImu = (robot.getEncoderSetup() == EncoderSetup.TWO_WHEELS);

        //Prep the timer object
        StopWatch travelLegTimer = new StopWatch();
        long loopEndTime, loopDuration = 0L;
        long loopStartTime = travelLegTimer.getElapsedTimeMillis();
        long timeLimit = calculateTimeLimitMillis(robot); //todo:  Verify calculation for timeLimit

        //prep loop variables
        int loopCount = 0;
        opMode.telemetry.clearAll();

        while(
                opMode.opModeIsActive()
                //&& loopCount < 500 //provide a safe out during debug
                && (!isTargetPoseReached(robot) && !isTimedOut(travelLegTimer, timeLimit))
        ) {
            loopCount++;

            if (debugOn) {
                Log.d(logTag, "____________Start of Loop _________________");
                logPosition(robot, loopCount, travelLegTimer);
            }

            //1) Calculate PoseError -- x,y, heading components of error and errorSums for integrator (using field coordinate system)
            //   a) Read in the Encoder Values (or simulate output if using virtual)
            robot.bulkReadSensorInputs(loopDuration);
            //   b) Update robot's field position based on readings
            this.updatePoseAfterLoop(robot);
            //   c) Calculate error
            robot.getPoseError().calculateError(robot, loopDuration, speed);

            //2) Compute the DriveCommand for the robot considering error & speed(in the robot's coordinate system)
            robot.setDriveCommand(new DriveCommand(robot, speed));
            if (debugOn) Log.d(logTag, robot.getDriveCommand().toString());

            //3) Calculate the motor powers based on DriveCommand
            robot.calculateDrivePowers();

            //4) Scale for softStart if necessary
            if(loopStartTime < softStart.getDurationMillis()){
                double softStartScale = softStart.getScaleFactor(loopStartTime);
                robot.applyScaleToCalculatedDrive(softStartScale);
            }

            //5) Apply the calculated motor powers
            if(debugOn){
                Log.d(logTag, "About to make robot Drive with " + robot.getDriveCommand().toString());
            }
            robot.drive();

            //  End the control loop here because loopDuration is needed for the Integral term
            loopEndTime = travelLegTimer.getElapsedTimeMillis();
            loopDuration = loopEndTime - loopStartTime;

            if (debugOn){
                StringBuilder sb = new StringBuilder();
                sb.append("Loop Start Time: ");
                sb.append(loopStartTime);
                sb.append("ms Loop End Time: ");
                sb.append(loopEndTime);
                sb.append("ms Loop Duration: ");
                sb.append(loopDuration);
                sb.append("ms");
                Log.d(logTag, sb.toString());
            }

            opMode.telemetry.addData("Actual Pose: ", robot.getActualPose().toString());
            opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
            opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
            opMode.telemetry.update();
            loopStartTime = loopEndTime;
            if (debugOn) Log.d(logTag, "____________End Loop " + loopCount + "_________________");
        }

        //Report out the status of the travel leg
        if (debugOn) {
            if (isTargetPoseReached(robot)) {
                Log.d(logTag, "Pose Achieved in " + format("%.2f", travelLegTimer.getElapsedTimeSeconds()));
            } else {
                Log.d(logTag, "Failed to reach target, timed out!!! " + robot.getPoseError().toString());
            }
        }

        robot.stop();
    }

    private boolean isTimedOut(StopWatch stopWatch, long timeLimit){
        return (stopWatch.getElapsedTimeMillis() < timeLimit) ? false: true;
    }

    public void updatePoseAfterLoop(Robot robot){
        /**
         * Update robot location based on the readings of encoders
         */

        boolean debugOn = true;
        String logTag = "EBOTS";
        if (debugOn) {
            Log.d(logTag, "Entering updatePoseAfterLoop");
            Log.d(logTag, (robot.toString()));
        }

        // Update the robot's position (uses PoseChange object)
        robot.updateActualPose();
        // Update the encoder currentClicks value for the next loop
        robot.updateAllSensorValues();
    }

    private long calculateTimeLimitMillis(Robot robot){
        //Find the expected time required to achieve the target pose
        boolean debugOn = true;
        String logTag = "EBOTS";
        if(debugOn) Log.d(logTag, "Entering calculateTimeLimitMillis...");

        double translateDistance = robot.getPoseError().getMagnitude();
        double rotationAngleRad = Math.abs(robot.getPoseError().getHeadingErrorRad());  //Don't allow negative value

        //Take the robot top Speed (in/s) and multiply by Speed object's top speed [0-1]
        double topTranslationSpeed = robot.getTopSpeed() * speed.getMaxSpeed();
        //Take robots top angular speed (rad/s) and multiply by Speed Objects top turn speed[0-1]
        double topSpinSpeed = robot.getAngularTopSpeedRad() * speed.getTurnSpeed();

        long translateTimeMillis = (long) ((translateDistance / topTranslationSpeed)*1000);
        long spinTimeMillis = (long) ((rotationAngleRad / topSpinSpeed)*1000);
        long bufferMillis = 1000L;
        long calculatedTime = (translateTimeMillis + spinTimeMillis + bufferMillis + softStart.getDurationMillis());

        if(debugOn) Log.d(logTag, "Calculated Time: " + String.format("%.2f", (float)(calculatedTime/1000)) + " s");
        return (calculatedTime);
    }
    private boolean isTargetPoseReached(Robot robot){
        boolean debugOn = true;
        String logTag  = "EBOTS";
        if (debugOn) Log.d(logTag, "Entering isTargetPoseReached...");

        double spinTolerance = accuracy.getHeadingAccuracyDeg();
        double positionTolerance = accuracy.getPositionalAccuracy();
        double integratorUnwindTolerance = accuracy.getIntegratorUnwindLimit();
        double spinIntegratorUnwindTolerance = accuracy.getSpinIntegratorUnwindLimit();

        boolean xPositionReached = (Math.abs(robot.getPoseError().getErrorComponent(CsysDirection.X)) > positionTolerance) ? false: true;
        boolean yPositionReached = (Math.abs(robot.getPoseError().getErrorComponent(CsysDirection.Y)) > positionTolerance) ? false: true;
        boolean spinTargetReached = (Math.abs(robot.getPoseError().getHeadingErrorDeg()) > spinTolerance) ? false : true;
        boolean xIntegratorUnwound = (Math.abs(robot.getPoseError().getErrorSumComponent(CsysDirection.X)) > integratorUnwindTolerance) ? false : true;
        boolean yIntegratorUnwound = (Math.abs(robot.getPoseError().getErrorSumComponent(CsysDirection.Y)) > integratorUnwindTolerance) ? false : true;
        boolean spinIntegratorUnwound = (Math.abs(robot.getPoseError().getHeadingErrorDegSum()) > spinIntegratorUnwindTolerance) ? false : true;

        if(debugOn) {
            String results = "xPos: " + xPositionReached + ", yPos: " + yPositionReached + ", spin: " + spinTargetReached;
            results = results + ", xInt: " + xIntegratorUnwound + ", yInt: " + yIntegratorUnwound + ", spinInt: " + spinIntegratorUnwound;
            Log.d(logTag, results);
        }

        if (xPositionReached && yPositionReached && spinTargetReached
                && xIntegratorUnwound && yIntegratorUnwound && spinIntegratorUnwound){
            return true;
        } else {
            return false;
        }
    }

    private static void logPosition(Robot robot, int loopCount, StopWatch timer){
        String logTag = "EBOTS";
        Log.d(logTag, "Logging Position...");

        Log.d(logTag, timer.toString(loopCount));
        Log.d(logTag, "Start Position " + robot.getActualPose().toString());
        Log.d(logTag, "Target Position " + robot.getTargetPose().toString());
        Log.d(logTag, "Error " + robot.getPoseError().toString());

    }

}
