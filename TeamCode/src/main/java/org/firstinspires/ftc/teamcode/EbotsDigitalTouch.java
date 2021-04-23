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
        DETECT_BACK_WALL("backWall"),
        SELECT_START_LINE("selectStartLine");
//        SELECT_DELAY("selectDelay");
        //SENSE_WOBBLE_GOAL("senseWobbleGoal");

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
        this.digitalTouch = hardwareMap.get(DigitalChannel.class, buttonFunctionIn.getDeviceName());
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public DigitalChannel getDigitalTouch(){return this.digitalTouch;}

    public ButtonFunction getButtonFunction() {
        return buttonFunction;
    }

    public boolean getIsPressed(){return this.isPressed;}

    public void setIsPressed(){

        // Note that backWall logic is reversed from other digital touch sensors
        // Rev digital touches are closed by default, this one is open be default
        if(this.buttonFunction==ButtonFunction.DETECT_BACK_WALL){
            this.isPressed = this.digitalTouch.getState();
        } else{
            this.isPressed = !this.digitalTouch.getState();
        }
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
