package org.firstinspires.ftc.teamcode.robotSubSystems.Elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Elevator {

    private static DcMotor elevatorMotor;
    private static double motorPower = 0;
    public static Servo elevatorServo;
    private static boolean servoToggle = false;
    private static boolean lastDPadRight = true;
    private static PID elevatorPID = new PID(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd, 0, 0);
    private static double height = 0;
    private static ElevatorState elevatorState = ElevatorState.CLOSED;


    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorServo = hardwareMap.get(Servo.class, "elevatorServo");


        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        height = 0;
    }

    public static void operate (ElevatorState state, Gamepad gamepad, Telemetry telemetry){

        height = elevatorMotor.getCurrentPosition();
        telemetry.addData("height", height);
        telemetry.update();

        switch (state){
            case CLIMB:
                elevatorPID.setWanted(ElevatorConstants.climbHeight);
                motorPower = elevatorPID.update(height);
                if (height < -1200 && !servoToggle){
                    servoToggle = true;
                }
                break;
            case CLOSED:
                elevatorPID.setWanted(ElevatorConstants.homeHeight);
                motorPower = elevatorPID.update(height);
                break;
            case STOP:
                motorPower = 0;
                break;
            case OVERRIDE:
                if (gamepad.dpad_up){
                    motorPower = ElevatorConstants.powerUp;
                } else if (gamepad.dpad_down){
                    motorPower = ElevatorConstants.powerDown;
                } else {
                    motorPower = 0;
                }
                break;
        }

        elevatorMotor.setPower(motorPower);

        if (gamepad.dpad_right && !lastDPadRight){
        servoToggle = !servoToggle;
}
        if (servoToggle){
            elevatorServo.setPosition(ElevatorConstants.servoOpen);
        } else {
            elevatorServo.setPosition(ElevatorConstants.servoClose);
        }
        lastDPadRight = gamepad.dpad_right;
        elevatorState = state;
    }

    public static void setPower (float power){
        elevatorMotor.setPower(power);            //for the override. this will be implemented in the subSystemManager
    }

    public static void firstTime(Gamepad gamepad, Telemetry telemetry){ //only for the first time for the configuration
        telemetry.addData("height", elevatorMotor.getCurrentPosition());
        telemetry.addData("height2", height);
        telemetry.update();
        height = elevatorMotor.getCurrentPosition();
        if (gamepad.right_bumper){
            elevatorMotor.setPower(ElevatorConstants.powerUp);
        } else if (gamepad.left_bumper){
            elevatorMotor.setPower(ElevatorConstants.powerDown);
        } else {
            elevatorMotor.setPower(0);
        }

        if (gamepad.dpad_right){
            servoToggle = !servoToggle;
        }
        if (servoToggle){
            elevatorServo.setPosition(ElevatorConstants.servoOpen);
        } else {
            elevatorServo.setPosition(ElevatorConstants.servoClose);
        }
    }

}
