package org.firstinspires.ftc.teamcode;

public class ErrorSum {
    private double value;
    private CsysDirection csysDirection;
    private CoordinateSystem coordinateSystem;

    public ErrorSum(CsysDirection dir){
        this.value = 0;
        this.csysDirection = dir;
        this.coordinateSystem = CoordinateSystem.FIELD;
    }

    public double getValue() {
        return value;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public void update(Robot robot, long loopDuration, Speed speed){
        //  Step 1:  See if integrator must be updated
        //           Add the integrator if the following conditions are met
        //              1) Loop Duration > 0s AND
        //              2) Either:
        //                  a) raw signal not saturated or
        //                  b) error is different sign than error sum

        //Check if the integrator is active for the current direction
        // (X or Y look at translate Integrator coefficient k_i, heading looks at Spin s_i)
        boolean isIntegratorOn = speed.isTranslateIntegratorOn();  //assume translate
        if(this.csysDirection == CsysDirection.Heading){
            isIntegratorOn = speed.isSpinIntegratorOn();        //update value if heading
        }

        //Set the value to zero if duration is 0 or integrator is off
        if (loopDuration <= 0 | !isIntegratorOn) {
            this.value = 0.0;
            return;
        }

        boolean signalSaturated = robot.getDriveCommand().isSignalSaturated(this.csysDirection);
        boolean sameSign = Math.signum(robot.getPoseError().getXError()) == Math.signum(this.value);


        if(!signalSaturated  | !sameSign){
            double currentError = robot.getPoseError().getErrorComponent(this.csysDirection);
            double timeDivisor = loopDuration * 1000.0;
            this.value += (currentError / timeDivisor);
        }
    }

}
