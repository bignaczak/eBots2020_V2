package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public interface AutonState {
    /**
     * AutonState is an interface defining common methods for autonomous states
     * It is used along with AutonStateFactory to generate instances of AutonStates
     * The enumeration AutonStateEnum controls the available options
     */

    public void performStateActions();

    public boolean areExitConditionsMet();

    public AutonStateEnum getNextAutonState();

    public AutonStateEnum getCurrentAutonStateEnum();
}
