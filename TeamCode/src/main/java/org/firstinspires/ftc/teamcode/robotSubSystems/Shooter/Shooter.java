package org.firstinspires.ftc.teamcode.robotSubSystems.Shooter;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Shooter {
    public abstract void init(HardwareMap hardwareMap);
    public abstract void operate(ShooterState state, Gamepad gamepad);
    public abstract void firstTime(Gamepad gamepad, Telemetry telemetry);
}
