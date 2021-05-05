package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateShootPowerShots implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;
    int ringsLaunched = 0;
    StopWatch ringLaunchTimer = new StopWatch();
    long ringCadence = 1500L;


    // ***********   CONSTRUCTOR   ***********************
    public StateShootPowerShots(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.SHOOT_POWER_SHOTS;
        this.nextAutonStateEnum = AutonStateEnum.PARK_ON_LAUNCH_LINE;
        stateTimeLimit = 10000;
        stateStopWatch = new StopWatch();
        robot.startLauncher();

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
        Log.d("EBOTS", "StateShootPowerShotes::areExitConditionsMet");
        return (ringsLaunched > 2 | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        Log.d("EBOTS", "StateShootPowerShotes::performStateSpecificTransitionActions");
        robot.stopLauncher();
        robot.stopConveyor();
        //Create a new target pose on the launch line in the center of field
//        double xCoord = (new LaunchLine()).getX() - (robot.getSizeCoordinate(CsysDirection.X) / 2);
//        Pose targetPose = new Pose(xCoord, 36, 0);
//        robot.setTargetPose(targetPose);
    }

    @Override
    public void performStateActions() {
        if(ringLaunchTimer.getElapsedTimeMillis() > ringCadence) {
            ringLaunchTimer.reset();
            robot.feedRing();
            ringsLaunched++;
            if (ringsLaunched == 2) {
                robot.startConveyor();
                ringCadence += 1500;
            }
//            if (ringsLaunched > 2) {
//                robot.startConveyor();
//                long feedTime = 1500;
//                long pauseTime = 500;
//                while (ringLaunchTimer.getElapsedTimeMillis() < feedTime) {
//                    //wait for ring to feed
//                }
//                robot.stopConveyor();
//                while(ringLaunchTimer.getElapsedTimeMillis() < (feedTime + pauseTime)){
//                    //let the ring setting in the launcher for a bit
//                }
//            }

        }
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
        opMode.telemetry.update();
    }
}
