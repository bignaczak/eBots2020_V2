package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StatePrematchSetup implements AutonState{

    LinearOpMode opMode;
    Robot robot;
    AutonStateEnum autonStateEnum;


    public StatePrematchSetup(LinearOpMode opModeIn, Robot robotIn){
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.autonStateEnum = AutonStateEnum.PREMATCH_SETUP;
    }

    @Override
    public void performStateActions() {

    }

    @Override
    public boolean areExitConditionsMet() {
        return false;
    }

    @Override
    public AutonStateEnum getNextAutonState() {
        return null;
    }

    @Override
    public AutonStateEnum getCurrentAutonStateEnum() {
        return this.autonStateEnum;
    }
}
