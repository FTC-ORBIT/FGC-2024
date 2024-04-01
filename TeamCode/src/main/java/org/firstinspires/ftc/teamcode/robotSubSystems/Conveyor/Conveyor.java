package org.firstinspires.ftc.teamcode.robotSubSystems.Conveyor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Conveyor {
    private static CRServo conveyorServo;
    private static double motorPower = 0;
    private static double servoPower = 0;
    private  static  double servoSwitchTime = 0;
    private  static  boolean forward = false;
    public static void init(HardwareMap hardwareMap){
        conveyorServo = hardwareMap.get(CRServo.class,"conveyorServo");

        conveyorServo.setDirection(DcMotorSimple.Direction.FORWARD);


         servoSwitchTime = 0;
    }
    public static void operate(ConveyorState state, Gamepad gamepad){
        switch (state){
            case TRANSPORT:
                servoPower = 1;
                break;
            case BACKWARDS:
            case INTAKE:
                servoPower = -1;
                break;
            case STOP:
                servoPower = 0;
                break;
            case OVERRIDE:
                servoPower = -gamepad.right_stick_x;
                break;
        }

        conveyorServo.setPower(servoPower);

    }
}
