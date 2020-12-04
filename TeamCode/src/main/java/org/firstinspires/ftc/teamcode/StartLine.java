package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class StartLine {
    /****************************************************************
     * *******      CLASS VARIABLES
     /****************************************************************/
    private FieldPosition fieldPosition;
    private ArrayList<SizeCoordinate> sizeCoordinates;

    /****************************************************************
     * *******      ENUMS
     /****************************************************************/
    public enum LinePosition {
        INNER(13.0),
        OUTER(37.0);

        private double yCenter;

        LinePosition(double yCenterIn){
            //Assigns xCenter assuming Blue Alliance
            this.yCenter = yCenterIn;
        }

        public double getyCenter() {
            return yCenter;
        }
    }
    /****************************************************************
     * *******      CONSTRUCTORS
     /****************************************************************/
    public StartLine(LinePosition linePosition, Alliance alliance){
        //Resolve the correct X position based on alliance and line position
        double allianceSign = (alliance == Alliance.BLUE) ? 1 : -1;  //Flip sign if red
        double yCenter = linePosition.getyCenter() * allianceSign;
        this.fieldPosition = new FieldPosition(12.0, yCenter, CoordinateSystem.FIELD);

        //Create the size coordinates
        this.sizeCoordinates.add(new SizeCoordinate(CsysDirection.X, 24.0));
        this.sizeCoordinates.add(new SizeCoordinate(CsysDirection.Y, 2.0));
    }

    /****************************************************************
     * *******      SIMPLE GETTERS
    /****************************************************************/

    public double getSizeCoordinate(CsysDirection dir){
        double sizeValue = 0;
        if(sizeCoordinates != null && dir != null && sizeCoordinates.size() > 0){
            sizeValue = SizeCoordinate.getSizeFromCoordinates(dir, sizeCoordinates);
        }
        return sizeValue;
    }

    public FieldPosition getFieldPosition() {
        return fieldPosition;
    }
}