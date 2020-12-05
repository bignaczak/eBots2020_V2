package org.firstinspires.ftc.teamcode;

public class PlayField {
    private final double fieldWidth = 144.0;
    private final double fieldHeight= 144.0;

    public PlayField(){}

    public double getFieldWidth() {
        return fieldWidth;
    }

    public double getFieldHeight() {
        return fieldHeight;
    }

    public double getYCoordTouchingWall(double headingAngle){
        double yCoord = -fieldHeight / 2;
        if(headingAngle == 0 | headingAngle == 180){
            yCoord += (Robot.RobotSize.xSize.getSizeValue()/2);
        } else if(yCoord == 90 | yCoord == -90){
            yCoord += (Robot.RobotSize.ySize.getSizeValue()/2);
        }
        return yCoord;
    }
}
