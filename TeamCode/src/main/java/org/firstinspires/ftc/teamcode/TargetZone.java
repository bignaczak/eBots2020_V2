package org.firstinspires.ftc.teamcode;

public class TargetZone {
    public Alliance alliance;
    public Zone zone;
    public double boxArea;
    public double boxPerimeter;
    public double boxLength;
    public double boxWidth;
    public double boxLeftEdge;
    public FieldPosition fieldPosition;


    //making enum alliance (Blue or Red)
//    public enum Alliance {
//        BLUE,
//        RED;
//    }
    //making enum zone (A,B or C)
    public enum Zone {
        A,
        B,
        C
    }

    public TargetZone(Alliance a,Zone z){
        this.alliance = a;
        this.zone = z;

        //assigning position on assuming blue alliance
        if(this.zone == Zone.A){
            this.fieldPosition = new FieldPosition(60,12);
        } else if (this.zone == Zone.B){
            this.fieldPosition = new FieldPosition(36,36);
        } else {
            this.fieldPosition = new FieldPosition(60,60);
        }

        //if on red alliance flip sign of y position
        if (alliance == Alliance.RED ){
            this.fieldPosition.yPosition = -this.fieldPosition.yPosition;
        }

        //assigning box Length, box Width, box size and box left edge
        this.boxWidth = 23.50;
        this.boxLength = 23.50;
        this.boxPerimeter = this.boxLength * 2 + this.boxWidth * 2;
        this.boxArea = this.boxLength * this.boxWidth;
        this.boxLeftEdge = this.fieldPosition.yPosition - this.boxWidth/2;
    }
    public FieldPosition getFieldPosition(){
        return this.fieldPosition;
    }
}