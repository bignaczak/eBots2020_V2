package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StatePickupSecondWobbleGoal implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum currentAutonStateEnum;
    AutonStateEnum nextAutonStateEnum;
    long stateTimeLimit;
    StopWatch stateStopWatch;
    int cranePos;

    // ***********   CONSTRUCTOR   ***********************
    public StatePickupSecondWobbleGoal(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.currentAutonStateEnum = AutonStateEnum.PICKUP_SECOND_WOBBLE_GOAL;
        this.nextAutonStateEnum = AutonStateEnum.MOVE_TO_TARGET_ZONE_AGAIN;
        stateTimeLimit = 2000L;
        stateStopWatch = new StopWatch();
        robot.toggleGripper();
        cranePos=robot.getCRANE_MIN_CRANE_HEIGHT();   // assumes the robot starts at the MIN height for grabbing
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
        boolean shouldExit = false;
        // Time limit is a dummy condition until manip mech ready
        if (cranePos <= robot.getCRANE_DRAG_HEIGHT() - 5 | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit |
                !opMode.opModeIsActive()){
            shouldExit = true;
        }
        return shouldExit;
    }

    @Override
    public void performStateSpecificTransitionActions() {

    }

    @Override
    public void performStateActions() {
        cranePos = robot.moveCraneToDragWobbleGoal();

        //report telemetry
        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
        opMode.telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
        opMode.telemetry.update();
    }
}
