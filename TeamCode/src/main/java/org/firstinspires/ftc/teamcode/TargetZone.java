package org.firstinspires.ftc.teamcode;

public class TargetZone {
    private Alliance alliance;
    public Zone zone;
    public double boxArea;
    public double boxPerimeter;
    public double boxLength;
    public double boxWidth;
    public double boxLeftEdge;
    public FieldPosition fieldPosition;


    //making enum zone (A,B or C)
    public enum Zone {
        A,
        B,
        C
    }
    public TargetZone(Alliance a,Zone z){
        this.alliance = a;
        this.zone = z;

        this.calculateFieldPosition(a, z);

    }
    public FieldPosition getFieldPosition(){
        return this.fieldPosition;
    }

    public void setAlliance(Alliance a){
        this.alliance = a;
        this.calculateFieldPosition(a, this.zone);
    }

    public void setZone(Zone z){
        this.zone = z;
        this.calculateFieldPosition(this.alliance, z);
    }
    private void calculateFieldPosition(Alliance a, Zone z){
        //assigning position on assuming blue alliance
        if(this.zone == Zone.A){
            this.fieldPosition = new FieldPosition(12,60);
        } else if (this.zone == Zone.B){
            this.fieldPosition = new FieldPosition(36,36);  // y was 36
        } else {
            this.fieldPosition = new FieldPosition(60,60);
        }

        //if on red alliance flip sign of y position
        if (alliance == Alliance.RED ){
            this.fieldPosition.setyPosition(-this.fieldPosition.getPositionComponent(CsysDirection.Y));
        }
        //assigning box Length, box Width, box size and box left edge
        this.boxWidth = 23.50;
        this.boxLength = 23.50;
        this.boxPerimeter = this.boxLength * 2 + this.boxWidth * 2;
        this.boxArea = this.boxLength * this.boxWidth;
        this.boxLeftEdge = this.fieldPosition.getyPosition() - this.boxWidth/2;
    }
}