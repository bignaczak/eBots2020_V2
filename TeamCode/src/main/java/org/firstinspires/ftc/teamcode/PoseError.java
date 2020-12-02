package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

import static java.lang.String.format;

public class PoseError {
    //  PoseError represents the difference in position and heading
    //  between a given pose and a target pose
    //  By default these are in FIELD coordinate system
    //  But may need to check the positionErrorVector coordinate system to verify
    //  This class provides a method to transform the error into the ROBOT's coordinate system

    /***************************************************************
    ******    CLASS VARIABLES
    ***************************************************************/
    private double headingErrorDeg;     //Error in which way the robot is facing
    private FieldPosition positionError;    //Field position object for X,Y error from robot's targetPose

    private ArrayList<ErrorSum> errorSums;      //Arraylist of all error Sums (X, Y, Spin)

    /***************************************************************
     ******    SIMPLE GETTERS AND SETTERS
     ***************************************************************/

    public double getXError() {return this.getErrorComponent(CsysDirection.X);}
    public double getYError() {return this.getErrorComponent(CsysDirection.Y);}
    public double getErrorComponent(CsysDirection dir){
        double errorValue = 0;

        if(dir == CsysDirection.X | dir == CsysDirection.Y) {
            errorValue = positionError.getPositionComponent(dir);
        }
        else if(dir == CsysDirection.Heading) {
            errorValue = headingErrorDeg;
        }
        return errorValue;
    }

    public double getHeadingErrorDeg() {return headingErrorDeg;}
    public double getHeadingErrorRad() {return Math.toRadians(headingErrorDeg);}

    public double getXErrorSum() {return getErrorSumComponent(CsysDirection.X);}
    public double getYErrorSum() {return getErrorSumComponent(CsysDirection.Y);}
    public double getHeadingErrorDegSum() {return getErrorSumComponent(CsysDirection.Heading);}

    public double getErrorSumComponent(CsysDirection dir){
        double errorSumValue = 0;
        for(ErrorSum errorSum: errorSums){
            if(errorSum.getCsysDirection() == dir){
                errorSumValue = errorSum.getValue();
                break;
            }
        }
        return errorSumValue;
    }
    /***************************************************************
     ******    CALCULATED PROPERTIES
     ***************************************************************/
    public double getMagnitude() {return this.positionError.getXYMagnitude(); }
    public double getFieldErrorDirectionDeg() {return this.positionError.getFieldErrorDirectionDeg();}

    /***************************************************************
    ******    CONSTRUCTORS
    ***************************************************************/
    public PoseError(){
        this.headingErrorDeg = 0;
        this.positionError = new FieldPosition(0,0,CoordinateSystem.FIELD);
        initializeErrorSums();
    }

    public PoseError(Robot robot) {
        //  Set the errorSum to zero when instantiated
        resetErrorSums();
        initializeError(robot);
    }

    /***************************************************************
    ******    CLASS INSTANCE METHODS
    ***************************************************************/

    public void resetErrorSums(){
        this.initializeErrorSums();
    }

    public void initializeError(Robot robot){
        //Initialize error
        this.positionError = new FieldPosition();
        calculateError(robot,0, Speed.SLOW);
        initializeErrorSums();
    }

    public void calculateError(Robot robot, long loopDuration, Speed speed){
        double xError = robot.getTargetPose().getX() - robot.getActualPose().getX();
        double yError = robot.getTargetPose().getY() - robot.getActualPose().getY();
        this.headingErrorDeg = robot.getTargetPose().getHeadingDeg() - robot.getActualPose().getHeadingDeg();
        this.positionError.setxPosition(xError);
        this.positionError.setyPosition(yError);

        //Now add the integrator if the loop duration is greater than 0
        if(loopDuration > 0) {
            updateErrorSums(robot, loopDuration, speed);
        }
    }

    private void initializeErrorSums(){
        errorSums = new ArrayList<>();
        //Loop through each direction
        for(CsysDirection dir:CsysDirection.values()){
            //Skip Z Direction
            if(dir != CsysDirection.Z){
                //Add a new errorSum object to the list
                errorSums.add(new ErrorSum(dir));
            }
        }
    }

    private void updateErrorSums(Robot robot, long loopDuration, Speed speed){
        for(ErrorSum errorSum:errorSums){
            errorSum.update(robot, loopDuration, speed);
        }
    }

    public FieldPosition getPositionErrorInRobotCoordinateSystem(Robot robot){
        //this is used in auton to determine how the robot must drive to achieve target pose

        //Note: the positionErrorVector is the distance between robot and target pose in FIELD coordinate system
        //Step 1:  Call the Coordinate System routine to perform the rotation transformation
        FieldPosition positionErrorInRobotCoordinateSystem = CoordinateSystem.transformCoordinateSystem(this.positionError,CoordinateSystem.ROBOT,robot);
        return positionErrorInRobotCoordinateSystem;
    }

    @Override
    public String toString(){
        return "xError [xErrorSum]: " + format("%.2f", this.getXError()) + " ["+ format("%.2f", this.getXErrorSum()) + "]" +
                "\n yError [yErrorSum]: " + format("%.2f", this.getYError()) + " ["+ format("%.2f", this.getYErrorSum()) + "]" +
                "\n spin Error [spinErrorSum]: " + format("%.2f", this.getHeadingErrorDeg()) + " ["+ format("%.2f", this.getHeadingErrorDegSum()) + "]";
    }

}
