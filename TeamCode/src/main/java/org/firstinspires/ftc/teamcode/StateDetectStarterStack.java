package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class StateDetectStarterStack implements AutonState{
    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    final boolean debugOn = true;
    final String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateDetectStarterStack(LinearOpMode opModeIn, Robot robotIn) {
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Instantiating class");

        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.DETECT_STARTER_STACK;
        this.nextAutonStateEnum = AutonStateEnum.INITIALIZE;
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
        }

        // Clear out the observations from previous runs
        try{
            if(StarterStackObservation.getObservationCount()>0) StarterStackObservation.clearObservations();
        } catch (Exception e){
            Log.d(logTag, "Error in StateDetectStarterStack constructor clearing obervations");
        }
    }


    // ***********   GETTERS   ***********************
    @Override
    public AutonStateEnum getNextAutonStateEnum() {
        return nextAutonStateEnum;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return currentAutonStateEnum;
    }


    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // This exits if the opMode is started
        if(debugOn) Log.d(logTag, " Exit conditions for DetectStarterStack: " + opMode.isStarted());
        return opMode.isStarted();
    }

    @Override
    public void performStateSpecificTransitionActions() {
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Entering performStateSpecificTransitionActions");

        if (tfod != null) {
            tfod.shutdown();
        }
    }

        @Override
    public void performStateActions() {
        //if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Entering performStateActions");

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            //if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Entering tfod detection");

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0 ){
                    new StarterStackObservation(TargetZone.Zone.A);
                }
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getHeight() < 40){
                        new StarterStackObservation(TargetZone.Zone.B);
                    } else {
                        //if the height is greater than 40 then the zone is C
                        new StarterStackObservation(TargetZone.Zone.C);
                    }
                }
                opMode.telemetry.addLine(StarterStackObservation.getObservationReport());
                opMode.telemetry.update();
            }
        }
    }


    // ***********   CLASS MEMBER METHODS   ***********************
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Entering initVuforia");

        String VUFORIA_KEY =
                "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        if(debugOn) Log.d(logTag, currentAutonStateEnum + ": Entering initTfod");

        String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
        String LABEL_FIRST_ELEMENT = "Quad";
        String LABEL_SECOND_ELEMENT = "Single";


        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
