package org.firstinspires.ftc.teamcode;

public class TowerGoal {
    //public double xPosition;
    //public double yPosition;
    public Alliance alliance;
    private FieldPosition fieldPosition;

    public TowerGoal(){
        this.alliance = Alliance.BLUE;
        this.fieldPosition = new FieldPosition(76, 36);
        this.fieldPosition.setyPosition(-this.fieldPosition.getyPosition());
    }

    public TowerGoal(Alliance a){
        this.fieldPosition = new FieldPosition(76, 36);
       this.alliance = a;
       if (a == Alliance.RED){
           this.fieldPosition.setyPosition(-this.fieldPosition.getyPosition());
       }
    }

    public FieldPosition getFieldPosition() {
        return fieldPosition;
    }
    public double getX(){
        return this.fieldPosition.getxPosition();
    }
    public double getY(){
        return this.fieldPosition.getyPosition();
    }
}
