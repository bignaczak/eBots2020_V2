package org.firstinspires.ftc.teamcode;

public enum EncoderCalibration {
    /**
     * This file contains calibration information for the encoders based on empirical findings
     */
    //Todo:  Add support for multiple robot configurations (Maybe RobotName:  TEST_BOT, 2020_BOT)
    FORWARD_RIGHT(3.099, 7.944),
    LATERAL(3.099, 7.944),
    FORWARD_LEFT(3.099, 3.888);

    private final double calibratedWheelDiameter;
    private final double calibratedSpinRadius;

    EncoderCalibration(double wheelDiam, double spinRadius){
        this.calibratedWheelDiameter = wheelDiam;
        this.calibratedSpinRadius = spinRadius;
    }

    public double getCalibratedWheelDiameter(){return calibratedWheelDiameter;}
    public double getCalibratedSpinRadius(){return calibratedSpinRadius;}

}
