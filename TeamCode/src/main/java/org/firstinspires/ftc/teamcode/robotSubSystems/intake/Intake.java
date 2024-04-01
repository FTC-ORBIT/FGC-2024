package org.firstinspires.ftc.teamcode.robotSubSystems.intake;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Intake {

    private static DcMotor intakeMotor;
    private static boolean forward = false;
    private static double intakeswitchTime = 0;


    public static void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeswitchTime = 0;
    }

    public static void operate(IntakeState state, Gamepad gamepad) {
        double power = 0;
        switch (state){
            case INTAKE:
                power = IntakeConstants.intakePower; //Needs to be checked on the robot
                break;
            case STOP:
                power = 0;
                break;
            case OVERRIDE:
                if (Math.abs(gamepad.right_stick_y) > 0.2) {
                    power = -gamepad.right_stick_y;
                }
                break;
            case SHOOTER_GREEN:

                if(GlobalData.currentTime - intakeswitchTime > 300){
                    forward = !forward;
                    intakeswitchTime = GlobalData.currentTime;
                }
                power = forward ? IntakeConstants.intakePower : -IntakeConstants.intakePower;
                break;
        }
        intakeMotor.setPower(power);
    }

    public static void firstTime(Gamepad gamepad){ //only for the first time for the configuration
        intakeMotor.setPower(gamepad.left_stick_y);
    }


}

