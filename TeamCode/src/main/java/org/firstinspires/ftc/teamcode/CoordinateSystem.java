package org.firstinspires.ftc.teamcode;

public enum CoordinateSystem {
    ROBOT, FIELD;

    public static FieldPosition transformCoordinateSystem(FieldPosition inputPosition, CoordinateSystem targetCoordinateSystem, Robot robot){
        //This performs a rotation transform of a Position and converts to either Field or Robot Coordinate System
        //It uses a calculation from https://en.wikipedia.org/wiki/Transformation_matrix#Rotation
        //Note:  this is typically used during auton to determine target distances from target using robot reference frame
        //  The inputPosition must have already been shifted so it's origin aligns with the robots origin  (find distance from robot to target point)
        //  Or when updating encoders to apply robot translation affects field position
        //  Demoed in ebots2020_PIDController_Simulation.xlsx
        if(inputPosition.getCoordinateSystem() == targetCoordinateSystem){
            return inputPosition;
        }
        double oldX = inputPosition.getxPosition();
        double oldY = inputPosition.getyPosition();
        double angleRad = robot.getActualPose().getHeadingRad();

        if (targetCoordinateSystem == CoordinateSystem.FIELD) {
            //flip the sign if switching to the Field coordinate system
            angleRad *= -1;
        }

        //these are from the transform equation
        double newX = oldX * Math.cos(angleRad) + oldY * Math.sin(angleRad);
        double newY = -oldX * Math.sin(angleRad) + oldY * Math.cos(angleRad);
        return new FieldPosition(newX, newY, targetCoordinateSystem);
    }

}
