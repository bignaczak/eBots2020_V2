package org.firstinspires.ftc.teamcode;

public class MovementComponent {
    private CoordinateSystem coordinateSystem;
    private CsysDirection csysDirection;
    private double distanceInches;

    public MovementComponent(CoordinateSystem csys, CsysDirection fd, double d){
        this.coordinateSystem = csys;
        this.csysDirection = fd;
        this.distanceInches = d;
    }

    public CoordinateSystem getCoordinateSystem() {
        return coordinateSystem;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public double getDistanceInches() {
        return distanceInches;
    }
}
