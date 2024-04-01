package org.firstinspires.ftc.teamcode.Sensors;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MagneticSensor {
    private TouchSensor magneticSensor;


    public MagneticSensor(HardwareMap hardwareMap, String name){
        magneticSensor = hardwareMap.get(TouchSensor.class, name);
    }

    public boolean getState(){
        return magneticSensor.isPressed();
    }
}
