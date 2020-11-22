package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class StartLine {
    private FieldPosition fieldPosition;
    private ArrayList<SizeCoordinate> sizeCoordinates;

    public enum LinePosition {
        Inner(13.0),
        Outer(37.0);

        private double xCenter;

        LinePosition(double xCenterIn){
            //Assigns xCenter assuming Blue Alliance
            this.xCenter = xCenterIn;
        }

        public double getxCenter() {
            return xCenter;
        }
    }

    public StartLine(LinePosition linePosition, Alliance alliance){
        //Resolve the correct X position based on allaince and line position
        double allianceSign = (alliance == Alliance.BLUE) ? 1 : -1;  //Flip sign if red
        double xCenter = linePosition.getxCenter() * allianceSign;
        this.fieldPosition = new FieldPosition(xCenter, 12.0, CoordinateSystem.FIELD);
        this.sizeCoordinates.add(new SizeCoordinate(CsysDirection.X, 24.0));
        this.sizeCoordinates.add(new SizeCoordinate(CsysDirection.Y, 2.0));
    }

    public double getSizeCoordinate(CsysDirection dir){
        double sizeValue = 0;
        if(sizeCoordinates != null && dir != null && sizeCoordinates.size() > 0){
            sizeValue = SizeCoordinate.getSizeFromCoordinates(dir, sizeCoordinates);
        }
        return sizeValue;
    }
}
