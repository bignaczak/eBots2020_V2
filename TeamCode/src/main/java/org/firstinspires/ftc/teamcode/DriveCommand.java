package org.firstinspires.ftc.teamcode;

class DriveCommand {
    private double magnitude;

    private double spin;
    private double angleRad;
    public DriveCommand(){
        this.magnitude = 0;
        this.spin = 0;
        this.angleRad = 0;
    }

    /**
     * GETTERS
     */
    public double getMagnitude() {
        return magnitude;
    }

    public double getSpin() {
        return spin;
    }
    public double getAngleRad() {
        return angleRad;
    }

    /**
     * SETTERS
     */
    public void setMagnitude(double magnitude) {
        this.magnitude = magnitude;
    }

    public void setSpin(double spin) {
        this.spin = spin;
    }

    public void setAngleRad(double angleRad) {
        this.angleRad = angleRad;
    }

}
