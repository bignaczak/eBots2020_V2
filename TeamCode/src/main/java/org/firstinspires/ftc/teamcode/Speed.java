package org.firstinspires.ftc.teamcode;

public enum Speed {
    SLOW (0.4, 0.2, 0.35, 0.15, 0.0, 0.05, 0.0, 0.0),
    MEDIUM (0.60,0.3,  0.10, 0.05, 0.0, 0.03, 0.0, 0.0),
    FAST (1.0, 0.4, 0.10, 0.05, 0.0,0.03, 0.0,0.0);

    /**  ENUM VARIABLES     **************/
    private double maxSpeed;
    private double turnSpeed;
    private double k_p;  //for translate proportional
    private double k_i;  //for translate integrator
    private double k_d;  //for translate derivative
    private double s_p;  //for spin proportional
    private double s_i;  //for spin integrator
    private double s_d;  //for spin derivative
    /**  CONSTRUCTOR    **************/
    Speed(double speed, double turnSpeed, double pGain, double iGain, double dGain, double spinPGain, double spinIGain, double spinDGain){
        this.maxSpeed = speed;
        this.turnSpeed = turnSpeed;
        this.k_p = pGain;
        this.k_i = iGain;
        this.k_d = dGain;
        this.s_p = spinPGain;
        this.s_i = spinIGain;
        this.s_d = spinDGain;
    }
    /**  ENUM GETTERS AND SETTERS  ***********/
    public double getMaxSpeed(){return this.maxSpeed;}
    public double getTurnSpeed(){return this.turnSpeed;}
    public double getK_p(){return this.k_p;}
    public double getK_i(){return this.k_i;}
    public double getS_p(){return this.s_p;}
    public double getS_i(){return this.s_i;}

    public void setK_i(double inputK_i){
        this.k_i = inputK_i;
    }

    public double getCoefficient(MotionControlCoefficient coef, CsysDirection dir){
        double coefficient = 0;
        if(dir == CsysDirection.X | dir == CsysDirection.Y){
            coefficient = (coef == MotionControlCoefficient.P) ? this.k_p : this.k_i;
        } else if(dir == CsysDirection.Heading){
            coefficient = (coef == MotionControlCoefficient.P) ? this.s_p : this.s_i;
        }
        return  coefficient;
    }

    public boolean isTranslateIntegratorOn(){
        boolean isTranslateIntegratorOn = false;
        if(this.k_i > 0){
            isTranslateIntegratorOn = true;
        }
        return isTranslateIntegratorOn;
    }

    public boolean isSpinIntegratorOn(){
        boolean isSpinIntegratorOn = false;
        if(this.s_i > 0){
            isSpinIntegratorOn = true;
        }
        return isSpinIntegratorOn;
    }

    @Override
    public String toString(){
        return "maxSpeed: " + maxSpeed + " , turnSpeed: " + turnSpeed
                + ", Translate Coefficients k_p / k_i / k_d: " + this.k_p + " / " + this.k_i + " / " + this.k_d
                + ", Spin Coefficients s_p / s_i / s_d: " + this.s_p + " / " + this.s_i + " / " + this.s_d;
    }
}

