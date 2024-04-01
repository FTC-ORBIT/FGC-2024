package org.firstinspires.ftc.teamcode.robotSubSystems.TankGrabber;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TankGrabber<ivate> {

    private static Servo tankGrabber;
    private static boolean lastRightBumper;
    private static boolean lastLeftBumper;
    private static double wantedPos = TankGrabberConstants.closedPos;
    private static double servoPos = 0;

    public static void init(HardwareMap hardwareMap) {
        tankGrabber = hardwareMap.get(Servo.class, "tankGrabber");
    }

    public static void operate(TankGrabberStates state) {
        switch (state) {
            case OPEN:
                wantedPos = TankGrabberConstants.openPos;
                break;
            case CLOSED:
                wantedPos = TankGrabberConstants.closedPos;
                break;

        }
//        tankGrabber.setPosition(wantedPos);
    }
    
    public static void firstTime(Gamepad gamepad, Telemetry telemetry) {

        if (gamepad.right_bumper && !lastRightBumper) {
            servoPos += 0.05;
        } else if (gamepad.left_bumper && !lastLeftBumper){
            servoPos -= 0.05;
        }

        tankGrabber.setPosition(servoPos);

        telemetry.addData("green", servoPos);

        lastRightBumper = gamepad.right_bumper;
        lastLeftBumper = gamepad.left_bumper;

    }

}

