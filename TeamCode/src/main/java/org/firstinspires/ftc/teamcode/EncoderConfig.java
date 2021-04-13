package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.HashMap;

public enum EncoderConfig {
    FOR_2019(RobotDesign.SEASON_2019),
    FOR_2020(RobotDesign.SEASON_2020);

    private DcMotorEx motor;
    private RobotOrientation robotOrientation;
    private EncoderTracker.SpinBehavior spinBehavior;          //
    private EncoderTracker.ClickDirection clickDirection;
    private EncoderModel encoderModel;
    private ArrayList<EncoderTracker> encoders;


    private static final HashMap<String, DriveWheel.WheelPosition> ENC_POS_2019 = new HashMap<String, DriveWheel.WheelPosition>(){
        {
            put("Forward", DriveWheel.WheelPosition.BACK_RIGHT);
            put("Lateral", DriveWheel.WheelPosition.FRONT_RIGHT);
            put("Third", DriveWheel.WheelPosition.FRONT_LEFT);

        }
    };

    private static final HashMap<String, DriveWheel.WheelPosition> ENC_POS_2020 = new HashMap<String, DriveWheel.WheelPosition>(){
        {
            put("Forward", DriveWheel.WheelPosition.FRONT_LEFT);
            put("Lateral", DriveWheel.WheelPosition.BACK_LEFT);
            put("Third", DriveWheel.WheelPosition.FRONT_LEFT);      //there are only 2 on this robot, so repeat

        }
    };



    EncoderConfig(RobotDesign robotDesign){

        // Create the first forward encoder

        
    }
}
