package org.firstinspires.ftc.teamcode;

public enum SoftStart {
    NO (false, 0L, 0.2),
    MEDIUM (true, 750L, 0.2),
    SLOW_START(true, 1200L, 0.2);

    /**  ENUM VARIABLES     **************/
    private boolean softStartOn;
    private long durationMillis;
    private double minPower;

    /**  CONSTRUCTOR    **************/
    SoftStart(boolean softStartOn, long durationMillis, double minPower){
        this.softStartOn = softStartOn;
        this.durationMillis = durationMillis;
        this.minPower = minPower;

    }
    /**  ENUM GETTERS AND SETTERS  ***********/
    public boolean isSoftStartOn(){return this.softStartOn;}
    public long getDurationMillis(){return  this.durationMillis;}
    public double getMinPower(){return minPower;}

    /**  ENUM Functions  ***********/
    public double getScaleFactor(long currentTimeMillis){
        //todo:  Consider if need to utilize minPower setting
        //       Currently, this ignores minPower and softStartOn
        double scaleFactor = 1.0;
        if(currentTimeMillis < durationMillis){
            scaleFactor = (double) (currentTimeMillis/durationMillis);
        }
        return scaleFactor;
    }
}
