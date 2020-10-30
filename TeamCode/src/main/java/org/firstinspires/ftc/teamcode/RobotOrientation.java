package org.firstinspires.ftc.teamcode;

public enum RobotOrientation {
    FORWARD(0), //Assumes increasing counts facing robot front
    LATERAL(Math.PI/2);  //Assumes increasing clicks facing robot left (top view)

    double encoderAngleRad;

    RobotOrientation(double inputAngleRad){
        this.encoderAngleRad = inputAngleRad;
    }
}
