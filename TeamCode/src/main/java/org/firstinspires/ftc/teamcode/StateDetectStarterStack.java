package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class StateDetectStarterStack implements AutonState{
    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;

    // ***********   CONSTRUCTOR   ***********************
    public StateDetectStarterStack(LinearOpMode opModeIn, Robot robotIn) {
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.DETECT_STARTER_STACK;
        this.nextAutonStateEnum = AutonStateEnum.INITIALIZE;
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
        return opMode.isStarted();
    }

    @Override
    public void performStateSpecificTransitionActions() {
        //set target position
        TargetZone.Zone observedTarget = StarterStackObservation.getObservedTarget();
        //todo could be deleted if not needed (line 225)
        TargetZone targetZone = new TargetZone(robot.getAlliance(), observedTarget);
        Pose targetPose = new Pose(targetZone.getFieldPosition(), 0);
        robot.setTargetPose(targetPose);
    }

        @Override
    public void performStateActions() {
        if (robot.getTfod() != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = robot.getTfod().getUpdatedRecognitions();
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
                opMode.telemetry.update();
            }
        }
    }
}
