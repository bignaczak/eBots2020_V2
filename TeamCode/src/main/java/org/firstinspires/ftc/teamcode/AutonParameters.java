package org.firstinspires.ftc.teamcode;

public enum AutonParameters {
    SIMULATED (Speed.SLOW, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.SLOW_START),
    DEBUG_TWO_WHEEL(Speed.SLOW, GyroSetting.EVERY_LOOP, Accuracy.STANDARD, SoftStart.SLOW_START),
    DEBUG_THREE_WHEEL(Speed.SLOW, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.SLOW_START),
    STANDARD_TW0_WHEEL(Speed.MEDIUM, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.MEDIUM),
    STANDARD_THREE_WHEEL(Speed.MEDIUM, GyroSetting.EVERY_LOOP, Accuracy.STANDARD, SoftStart.MEDIUM);

    private Speed speed;
    private GyroSetting gyroSetting;
    private Accuracy accuracy;
    private SoftStart softStart;

    AutonParameters(Speed speedIn, GyroSetting gyroIn, Accuracy accuracyIn, SoftStart softStartIn){
        this.speed = speedIn;
        this.gyroSetting = gyroIn;
        this.accuracy = accuracyIn;
        this.softStart = softStartIn;
    }

    public Speed getSpeed() {
        return speed;
    }

    public GyroSetting getGyroSetting() {
        return gyroSetting;
    }

    public Accuracy getAccuracy() {
        return accuracy;
    }

    public SoftStart getSoftStart() {
        return softStart;
    }
}
