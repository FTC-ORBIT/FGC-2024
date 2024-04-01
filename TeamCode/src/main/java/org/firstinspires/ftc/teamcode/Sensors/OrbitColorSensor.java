package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitColorSensor {

    private final ColorSensor colorSensor;

    public OrbitColorSensor(HardwareMap hardwareMap, String name) {
        colorSensor = hardwareMap.get(ColorSensor.class, name);
    }


    public boolean isShootingBlueBall() {
            float[] currentColors = {colorSensor.red(), colorSensor.green(), colorSensor.blue()};
            int check = 0;
            for (int i = 0; i < 3; i++){
                if ((currentColors[i] < Constants.blueBalls[i] + Constants.colorRange) && (currentColors[i] > Constants.blueBalls[i] - Constants.colorRange)){
                    check ++;
                }
            }
            if (check == 3) {
                check = 0;
                return true;
            } else {
                check = 0;
                return false;
            }

    }

    public void printRGB (Telemetry telemetry){
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
    }

}
