package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TouchSensor {

    DigitalChannel touchSensor;

    public TouchSensor (HardwareMap hardwareMap, String name){
        touchSensor = hardwareMap.get(DigitalChannel.class, name);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState (){
        return touchSensor.getState();
    }
}
