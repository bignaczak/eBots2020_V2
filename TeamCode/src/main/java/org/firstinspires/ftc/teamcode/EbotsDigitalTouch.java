package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class EbotsDigitalTouch {

    private DigitalChannel digitalTouch;
    private ButtonFunction buttonFunction;

    public enum ButtonFunction{
        SELECT_ALLIANCE("selectAlliance"),
        SELECT_START_LINE("selectStartLine"),
        SENSE_WOBBLE_GOAL("senseWobbleGoal");

        private String deviceName;

        ButtonFunction(String deviceNameIn){
            this.deviceName = deviceNameIn;
        }

        public String getDeviceName() {
            return deviceName;
        }
    };

    public EbotsDigitalTouch(ButtonFunction buttonFunctionIn,HardwareMap hardwareMap){
        this.buttonFunction = buttonFunctionIn;
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        this.digitalTouch = hardwareMap.get(DigitalChannel.class, buttonFunctionIn.getDeviceName());
    }

    public DigitalChannel getDigitalTouch(){return this.digitalTouch;}

    public ButtonFunction getButtonFunction() {
        return buttonFunction;
    }

    public static DigitalChannel getDigitalChannelByButtonFunction(ButtonFunction buttonFunction, ArrayList<EbotsDigitalTouch> digitalTouches){
        DigitalChannel returnObject = null;
        for(EbotsDigitalTouch edt: digitalTouches){
            if(edt.getButtonFunction() == buttonFunction){
                returnObject = edt.getDigitalTouch();
                break;
            }
        }
        return returnObject;
    }

}
