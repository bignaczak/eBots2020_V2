package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class EbotsRevBlinkinLedDriver {

    RevBlinkinLedDriver revBlinkinLedDriver;
    LedLocation ledLocation;
    RevBlinkinLedDriver.BlinkinPattern alliancePattern;
    RevBlinkinLedDriver.BlinkinPattern currentPattern;
    StopWatch patternTimer;




    public enum LedLocation{
        MAIN("blinkin");

        String deviceName;
        LedLocation(String deviceNameIn){
            this.deviceName = deviceNameIn;
        }

        public String getDeviceName(){return this.deviceName;}
    }

    public LedLocation getLedLocation(){return this.ledLocation;}
    public RevBlinkinLedDriver.BlinkinPattern getPattern = this.currentPattern;

    public EbotsRevBlinkinLedDriver(LedLocation ledLocation, Alliance alliance, HardwareMap hardwareMap){
        setAlliancePattern(alliance);
        this.ledLocation = ledLocation;
        this.revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, ledLocation.getDeviceName());
    }

    public void setAlliancePattern(Alliance alliance){
        if(alliance == Alliance.RED){
            this.alliancePattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        } else{
            this.alliancePattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        this.setPattern(alliancePattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){

        this.currentPattern = pattern;
        this.revBlinkinLedDriver.setPattern(currentPattern);
    }

    public static EbotsRevBlinkinLedDriver getEbotsRevBlinkinLedDriverByLedLocation(
            LedLocation ledLocation, ArrayList<EbotsRevBlinkinLedDriver> ebotsRevBlinkinLedDrivers){

        EbotsRevBlinkinLedDriver returnDriver = null;
        for(EbotsRevBlinkinLedDriver driver: ebotsRevBlinkinLedDrivers){
            if(driver.ledLocation == ledLocation){
                returnDriver = driver;
                break;
            }
        }
        return returnDriver;
    }

}
