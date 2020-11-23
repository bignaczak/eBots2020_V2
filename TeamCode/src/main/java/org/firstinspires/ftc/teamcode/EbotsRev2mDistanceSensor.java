package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EbotsRev2mDistanceSensor {

    private Rev2mDistanceSensor rev2mDistanceSensor;
    private RobotSide robotSide;

    public enum DistanceSensorName {
        //Enum for mapping sensors to the hardwareMap using name
        //This is a bit redundant because assuming one per RobotSide
        F("frontDistSensor"),
        R("rightDistSensor"),
        B("backDistSensor"),
        L("leftDistSensor");

        private String deviceName;

        DistanceSensorName(String deviceNameIn){
            this.deviceName = deviceNameIn;
        }

        //This is the primary function that is used
        public static String getSensorName(RobotSide robotSide){
            String returnName;
            if(robotSide == RobotSide.FRONT) {
                returnName = DistanceSensorName.F.getDeviceName();
            } else if(robotSide == RobotSide.RIGHT){
                returnName = DistanceSensorName.R.getDeviceName();
            }else if(robotSide == RobotSide.BACK){
                returnName = DistanceSensorName.B.getDeviceName();
            }else {
                returnName = DistanceSensorName.L.getDeviceName();
            }
            return returnName;
        }

        public String getDeviceName() {
            return deviceName;
        }
    }

    public EbotsRev2mDistanceSensor(RobotSide robotSide, HardwareMap hardwareMap){
        this.robotSide = robotSide;
        this.rev2mDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, DistanceSensorName.getSensorName(robotSide));
    }

    public Rev2mDistanceSensor getRev2mDistanceSensor() {
        return rev2mDistanceSensor;
    }

    public RobotSide getRobotSide() {
        return robotSide;
    }
}
