package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveByAprilTags.Camera;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.MagneticSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistanceSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.Sensors.Potentiometer;
import org.firstinspires.ftc.teamcode.Sensors.TouchSensor;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.OrbitLED;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainTank.DriveTrainTank;


@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {


    public static TelemetryPacket packet;


    @Override
    public void runOpMode() throws InterruptedException {


        FtcDashboard dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        ElapsedTime robotTime = new ElapsedTime();
        robotTime.reset();
        DrivetrainOmni.init(hardwareMap);
//        DriveTrainTank.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        Camera.initAprilTag(hardwareMap,telemetry);
//        OrbitColorSensor.init(hardwareMap);
//        OrbitDistanceSensor.OrbitDistanceSensor(hardwareMap);
//        MagneticSensor.MagneticSensor(hardwareMap,"magneticSensor");
//        Potentiometer.Potentiometer(hardwareMap);
//        TouchSensor.TouchSensor(hardwareMap,"touchSensor");
        OrbitGyro.resetGyroStartTeleop((float) Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.update();
        telemetry.addData("gyro", Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.update();

        GlobalData.inAutonomous = false;
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;


        waitForStart();

        GlobalData.robotState = RobotState.TRAVEL;

        while (!isStopRequested()) {
          GlobalData.currentTime = (float) robotTime.seconds();
          Vector leftStick = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y);
          float omega = gamepad1.right_trigger - gamepad1.left_trigger;
          DrivetrainOmni.operate(leftStick, omega , gamepad1);
//          DriveTrainTank.operate(-gamepad1.left_stick_y, gamepad1.right_trigger, gamepad1.left_trigger, telemetry, gamepad1);
          SubSystemManager.setSubsystemToState(gamepad1 , gamepad2);
           GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
           Camera.update();
            GlobalData.lastTime = GlobalData.currentTime;

            SubSystemManager.printStates(telemetry);
        }
    }



}
//dani yalechan!
// yoel yalechan!