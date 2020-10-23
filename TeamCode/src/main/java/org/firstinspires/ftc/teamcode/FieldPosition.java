package org.firstinspires.ftc.teamcode;

public class FieldPosition {
    public double yPosition;
    public double xPosition;
    public double zPosition;

    public FieldPosition(){
        this.xPosition = 0.0;
        this.yPosition = 0.0;
        this.zPosition = 0.0;
    }
    public FieldPosition(double x,double y){
            this.xPosition = x;
            this.yPosition = y;
            this.zPosition = 0.0;
    }

    public FieldPosition(double x,double y, double z){
        this(x,y);
        this.zPosition = z;
    }



}
