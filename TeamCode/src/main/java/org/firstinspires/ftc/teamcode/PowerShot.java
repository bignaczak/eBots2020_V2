package org.firstinspires.ftc.teamcode;

public class PowerShot {

    public Alliance alliance;
    public TargetPosition targetPosition;
    private FieldPosition fieldPosition;


    public enum TargetPosition{
        ONE, TWO, THREE
    }

    public PowerShot(TargetPosition tp, Alliance a) {
        this.alliance = a;
        this.targetPosition = tp;
        this.fieldPosition = new FieldPosition(73, 0, 23.5);
        if(tp == TargetPosition.ONE){
            this.fieldPosition.yPosition = 3.5;
        }else if(tp == TargetPosition.TWO){
            this.fieldPosition.yPosition = 11.0;
        }else{
            this.fieldPosition.yPosition = 18.5;
        }

        if(a == Alliance.RED){
            this.fieldPosition.yPosition *= -1;
        }

    }
    public FieldPosition getFieldPosition(){
        return this.fieldPosition;
    }
}

