package org.firstinspires.ftc.teamcode.Sensors;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MagneticSensor {
    private static TouchSensor magneticSensor;


    public static void MagneticSensor(HardwareMap hardwareMap, String name){
        magneticSensor = hardwareMap.get(TouchSensor.class, name);
    }

    public static boolean getState(){
        return magneticSensor.isPressed();
    }
}