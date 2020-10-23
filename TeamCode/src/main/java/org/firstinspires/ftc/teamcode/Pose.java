package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

/**
 * Pose represents the robots position AND heading relative to field coordinates
 * It can also carry instructions on what to do after an auton travel leg
 */
public class Pose {

    /****************************************************************
    //******    CLASS VARIABLES
    //***************************************************************/

    private double heading;         // Degrees, 0 inline with x-axis, positive CCW when viewed from top
    private FieldPosition fieldPosition;    // x, y and z position on field in inches.  0 is center of field.  x+ towards target goals.  y+ towards blue alliance side.
    /***************************************************************
    //******    STATIC VARIABLES
    //****************************************************************/
    private static final double overallFieldLength = 72.0;  //in inches


    /***************************************************************
    //******    ENUMERATIONS
    //***************************************************************/
    public enum StartingPose{
        //These starting poses assume BLUE Alliance
        INNER (-61.75, 25.0, 0.0)
        , OUTER (-61.75, 49.0, 0.0);

        private double xStart;
        private double yStart;
        private double headingStart;

        StartingPose(double xInput, double yInput, double headingInput){
            this.xStart = xInput;
            this.yStart = yInput;
            this.headingStart = headingInput;
        }

        public double getXStart() {
            return xStart;
        }
        public double getYStart() {
            return yStart;
        }
        public double getHeadingStart() {
            return headingStart;
        }
    }

    //***************************************************************88

    /*****************************************************************
    //******    CONSTRUCTORS
    //****************************************************************/

    public Pose(){
        //this.x = 0.0;
        //this.y = 0.0;
        this.fieldPosition = new FieldPosition();
        this.heading = 0.0;
    }     //Default constructor

    public Pose(double xInput, double yInput, double headingInput){
        this.fieldPosition = new FieldPosition(xInput,yInput);
        //this.x = xInput;
        //this.y = yInput;
        this.heading = headingInput;
    }

    public Pose(FieldPosition fp, double headingInput){
        this.fieldPosition = fp;
        this.heading = headingInput;
    }


    //  When using a pre-defined StartingPose from the enumeration
    public Pose(StartingPose startingPose, Alliance alliance) {
        this(startingPose.getXStart(), startingPose.getYStart(), startingPose.getHeadingStart());
        //Now flip the sign of the y component if on the red alliance
        if(alliance == Alliance.RED){
            this.fieldPosition.yPosition = -this.fieldPosition.yPosition;
        }
    }



    /*****************************************************************
    //******    SIMPLE GETTERS AND SETTERS
    //****************************************************************/
    public double getX() {
        return this.fieldPosition.xPosition;
    }
    public void setX(double x) {
        this.fieldPosition.xPosition = x;
    }
    public double getY() {
        return this.fieldPosition.yPosition;
    }
    public void setY(double y) {
        this.fieldPosition.yPosition = y;
    }
    public double getHeading() {
        return heading;
    }
    public double getHeadingRad(){
        return Math.toRadians(heading);
    }


    public void setHeading(double heading) {
        /** Sets heading, but makes sure it is within the legal bounds
         *  which is -180 < heading <= 180
         */
        //heading = TrackingPose.applyAngleBound(heading);
        this.heading = heading;
    }


    /***************************************************************88
    //******    Class METHODS
    //***************************************************************/

    @Override
    public String toString(){
        return "(" + String.format("%.2f",fieldPosition.xPosition) + " ," + String.format("%.2f",fieldPosition.yPosition) + " @ "
                + String.format("%.2f",heading) + ")";
    }

    /***************************************************************88
     //******    Static METHODS
     //***************************************************************/

}
