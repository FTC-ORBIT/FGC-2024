package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainTank;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveByAprilTags.Camera;

public class DriveTrainTank {
    private static final DcMotor[] motors = new DcMotor[2];
    static ElapsedTime time = new ElapsedTime();
    private static float omega = 0;
    private static float drive = 0;
//    private  static AutoDrive autoDrive = new AutoDrive();

    public static void init(HardwareMap hardwareMap) {
        time.reset();
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        //TODO make sure to reverse the correct motors according to your robot

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        drive = 0;
        omega = 0;

    }


    public static void operate(float y_Power, float right_trigger, float left_trigger, Telemetry telemetry, Gamepad gamepad1) {
//        final Vector addition = autoDrive.getAutoDriveAddition(telemetry, gamepad);
//        omega = (right_trigger - left_trigger);// + addition.y;
//        drive = y_Power;// + addition.x;
        float lMotorPower = (y_Power + left_trigger - right_trigger);
        float rMotorPower = (y_Power + right_trigger - left_trigger);
        if (gamepad1.left_bumper){
            Camera.getAprilTagDetectionTank();
        }else {
            motors[0].setPower(lMotorPower);
            motors[1].setPower(rMotorPower);
        }
    }

    public static void firstTime(Gamepad gamepad){  //only for the first time for the configuration
        motors[0].setPower(gamepad.left_stick_y); //leftMotor
        motors[1].setPower(gamepad.right_stick_y); //rightMotor
    }
}


