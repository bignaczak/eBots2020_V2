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
            this.fieldPosition.setyPosition(3.5);
        }else if(tp == TargetPosition.TWO){
            this.fieldPosition.setyPosition(11.0);
        }else{
            this.fieldPosition.setyPosition(18.5);
        }

        if(a == Alliance.RED){
            this.fieldPosition.setyPosition(-this.fieldPosition.getyPosition());
        }

    }
    public FieldPosition getFieldPosition(){
        return this.fieldPosition;
    }
}

