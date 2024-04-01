package org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.ShooterBlueBalls;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterBlueBallsConstants {
    public static double shooterPower = 0.65f;
    public static double faultLimit = 7f;  //this is the min voltage for shooting
    public static double voltageDownWhenShooting = 0.2; //this is how much the voltage goes down when there is a ball
    public static double faultMinTime = 500f; //this is the minimum time in milliseconds it takes to shot one ball after another
    public static double minTimeSec = 0.5; //as discussed, the min time between each set of 3 balls is 0.5 seconds for now
    public static double openDoorPos = 0.45;
    public static double closedDoorPos = 0.05;
    public static double shooterDelaySec = 1500;
    public static double preActiveSec = 1500;

}