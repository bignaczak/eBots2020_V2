package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class EbotsColorSensor {


    private int redColor;
    private int blueColor;
    private int greenColor;
    private ColorSensor colorSensor;
    public SensorLocation sensorLocation;
    final double SCALE_FACTOR = 255;

    public enum TapeColor {
        RED,
        BLUE,
        WHITE
    }

    public enum RobotSide {
        LEFT_SIDE,
        RIGHT_SIDE,
        FRONT_SIDE,
        BACK_SIDE
    }

    public enum SensorLocation {
        FRONT_LEFT("frontLeftSensorColor"),
        FRONT_RIGHT("frontRightSensorColor"),
        BACK_LEFT("backLeftSensorColor"),
        BACK_RIGHT("backRightSensorColor");

        private String deviceName;

        SensorLocation(String deviceNameIn) {
            deviceName = deviceNameIn;
        }

        public String getDeviceName() {
            return deviceName;
        }
    }

    public static EbotsColorSensor getEbotsColorSensor(SensorLocation sensorLocation, ArrayList<EbotsColorSensor> ebotsColorSensors) {
        EbotsColorSensor returnSensor = null;
        for (EbotsColorSensor sensor : ebotsColorSensors) {
            if (sensorLocation == sensor.sensorLocation) {
                returnSensor = sensor;
                break;
            }
        }
        return returnSensor;
    }

    public EbotsColorSensor(SensorLocation sensorLocationIn, HardwareMap hardwareMap) {
        this.sensorLocation = sensorLocationIn;
        this.colorSensor = hardwareMap.get(ColorSensor.class, sensorLocationIn.getDeviceName());

    }

    public void setColorValue() {
        redColor = (int) (colorSensor.red() * SCALE_FACTOR);
        blueColor = (int) (colorSensor.blue() * SCALE_FACTOR);
        greenColor = (int) (colorSensor.green() * SCALE_FACTOR);
    }

    public boolean isRed() {
        int redThreshold = 130;
        int redOther = 40;
        boolean returnValue = false;
        if (redColor >= redThreshold && blueColor <= redOther && greenColor <= redOther) {
            returnValue = true;
        }
        return returnValue;
    }

    public boolean isBlue() {
        int blueThreshold = 170;
        int blueOther = 40;
        boolean returnValue = false;
        if (blueColor >= blueThreshold && redColor <= blueOther && greenColor <= blueOther) {
            returnValue = true;
        }
        return returnValue;
    }

    public boolean isWhite() {
        int whiteThreshold = 200;
        boolean returnValue = false;
        if (redColor >= whiteThreshold && blueColor >= whiteThreshold && greenColor >= whiteThreshold) {
            returnValue = true;
        }
        return returnValue;
    }

    public boolean isColor(TapeColor tapeColor) {
        boolean returnValue = false;
        if(tapeColor == TapeColor.WHITE){
            returnValue = this.isWhite();
        }else if(tapeColor == TapeColor.BLUE){
            returnValue = this.isBlue();
        }else if(tapeColor == TapeColor.RED){
            returnValue = this.isRed();
        }
        return returnValue;
    }


    public static boolean isSideOnColor(ArrayList<EbotsColorSensor> ebotsColorSensors,RobotSide robotSide, TapeColor tapeColor){
        boolean returnValue = true;
        ArrayList<SensorLocation> sensorLocations = getSensorLocationsForSide(robotSide);
        for(EbotsColorSensor ecs: ebotsColorSensors) {
            if (sensorLocations.contains(ecs.sensorLocation)){
                if (!ecs.isColor(tapeColor)){
                    returnValue = false;
                }
            }
        }
        return returnValue;
    }
    private static ArrayList<SensorLocation> getSensorLocationsForSide(RobotSide robotSide) {
        ArrayList<SensorLocation> sensorLocations = new ArrayList<>();
        if (robotSide == RobotSide.FRONT_SIDE) {
            sensorLocations.add(SensorLocation.FRONT_LEFT);
            sensorLocations.add(SensorLocation.FRONT_RIGHT);
        } else if (robotSide == RobotSide.LEFT_SIDE) {
            sensorLocations.add(SensorLocation.FRONT_LEFT);
            sensorLocations.add(SensorLocation.BACK_LEFT);
        } else if (robotSide == RobotSide.RIGHT_SIDE) {
            sensorLocations.add(SensorLocation.FRONT_RIGHT);
            sensorLocations.add(SensorLocation.BACK_RIGHT);
        } else if (robotSide == RobotSide.BACK_SIDE) {
            sensorLocations.add(SensorLocation.BACK_RIGHT);
            sensorLocations.add(SensorLocation.BACK_LEFT);
        }
        return sensorLocations;
    }
}
