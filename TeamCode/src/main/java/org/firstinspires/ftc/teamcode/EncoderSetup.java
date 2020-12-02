package org.firstinspires.ftc.teamcode;

public enum EncoderSetup {
    TWO_WHEELS(RobotOrientation.NONE, EncoderModel.CTR)
    , THREE_WHEELS(RobotOrientation.FORWARD, EncoderModel.CTR);

    private RobotOrientation doubleEncoderDirection;
    private EncoderModel encoderModel;

    EncoderSetup(RobotOrientation robotOrientation, EncoderModel encoderModelIn){
        //Note: returns null if two wheel
        this.doubleEncoderDirection = robotOrientation;
        this.encoderModel = encoderModelIn;
    }

    public RobotOrientation getDoubleEncoderDirection() {return doubleEncoderDirection;}

    public EncoderModel getEncoderModel() { return encoderModel; }
}
