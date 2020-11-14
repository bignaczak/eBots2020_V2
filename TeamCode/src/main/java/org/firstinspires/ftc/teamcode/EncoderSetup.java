package org.firstinspires.ftc.teamcode;

public enum EncoderSetup {
    TWO_WHEELS(RobotOrientation.NONE)
    , THREE_WHEELS(RobotOrientation.FORWARD);

    private RobotOrientation doubleEncoderDirection;

    EncoderSetup(RobotOrientation robotOrientation){
        //Note: returns null if two wheel
        this.doubleEncoderDirection = robotOrientation;
    }

    public RobotOrientation getDoubleEncoderDirection() {return doubleEncoderDirection;}
}
