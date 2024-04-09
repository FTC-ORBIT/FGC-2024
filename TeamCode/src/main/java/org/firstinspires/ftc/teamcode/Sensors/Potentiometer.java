package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Potentiometer {

    private static AnalogInput potentiometer;

    public static void Potentiometer(HardwareMap hardwareMap){
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    public static double getVolt(){
        return potentiometer.getVoltage();
    }
}
