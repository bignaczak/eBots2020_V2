package org.firstinspires.ftc.teamcode;

public class TowerGoal {
    //public double xPosition;
    //public double yPosition;
    public Alliance alliance;
    private FieldPosition fieldPosition;

    public TowerGoal(Alliance a){
        this.fieldPosition = new FieldPosition(76, 36);
        //xPosition = 76;
        //yPosition = 36;
       this.alliance = a;
       if (a == Alliance.RED){
           this.fieldPosition.yPosition = -this.fieldPosition.yPosition;
       }
    }

    public FieldPosition getFieldPosition() {
        return fieldPosition;
    }
}
