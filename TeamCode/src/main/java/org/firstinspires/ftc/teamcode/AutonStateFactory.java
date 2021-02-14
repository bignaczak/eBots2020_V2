package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStateEnum;

public class AutonStateFactory {
    /**
     * This class acts as a factory for AutonStates which implement the AutonState interface
     */

    public AutonState getAutonState(AutonStateEnum autonStateEnum, LinearOpMode opMode, Robot robot){
        AutonState returnState = null;

        if(autonStateEnum == null) {
            returnState = null;
        }

        if(autonStateEnum==AutonStateEnum.PREMATCH_SETUP){
            returnState = new StatePrematchSetup(opMode, robot);
        }

        return returnState;
    }


}
