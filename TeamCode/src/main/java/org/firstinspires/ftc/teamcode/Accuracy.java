package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

public enum Accuracy{
    LOOSE (10.0, 3.0, 0.40, 0.10),
    STANDARD (4.0, 1.5, 0.20, 0.10),
    TIGHT (1.0, 0.50, 0.10, 0.10);

    /**  ENUM VARIABLES     **************/

    private double headingAngleAccuracy;
    private double positionalAccuracy;
    private double integratorUnwindLimit;
    private double spinIntegratorUnwindLimit;

    /**  CONSTRUCTOR    **************/

    Accuracy(double headingAngleAccuracy, double positionalAccuracy, double integratorUnwind, double spinIntegratorUnwind){
        this.headingAngleAccuracy = headingAngleAccuracy;
        this.positionalAccuracy = positionalAccuracy;
        this.integratorUnwindLimit = integratorUnwind;  // for translate position
        this.spinIntegratorUnwindLimit = spinIntegratorUnwind;  // for translate position
    }

    /**  ENUM GETTERS AND SETTERS  ***********/

    public double getHeadingAngleAccuracy() {
        return headingAngleAccuracy;
    }
    public double getPositionalAccuracy() {
        return positionalAccuracy;
    }
    public double getIntegratorUnwindLimit() {
        return integratorUnwindLimit;
    }
    public double getSpinIntegratorUnwindLimit() {
        return spinIntegratorUnwindLimit;
    }
    @Override
    public String toString(){
        return "Translate position tol: " + format("%.1f",positionalAccuracy)
                + ", Translate Integrator: " + format("%.1f", integratorUnwindLimit)
                + ", Heading angle tol: " + format("%.1f", headingAngleAccuracy)
                + ", Heading Integrator: " + format("%.1f", spinIntegratorUnwindLimit);
    }
}
