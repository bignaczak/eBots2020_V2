package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class EbotsDigitalTouch {

    private DigitalChannel digitalTouch;
    private ButtonFunction buttonFunction;
    private boolean isPressed;

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

    public boolean getIsPressed(){return this.isPressed;}

    public void setIsPressed(){
        this.isPressed = !this.digitalTouch.getState();
    }

    public static EbotsDigitalTouch getEbotsDigitalTouchByButtonFunction(ButtonFunction buttonFunction, ArrayList<EbotsDigitalTouch> digitalTouches){
        EbotsDigitalTouch returnObject = null;
        for(EbotsDigitalTouch edt: digitalTouches){
            if(edt.getButtonFunction() == buttonFunction){
                returnObject = edt;
                break;
            }
        }
        return returnObject;
    }

}
