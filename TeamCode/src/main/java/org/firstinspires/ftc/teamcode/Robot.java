package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Resolve;

import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.Conveyor.Conveyor;
import org.firstinspires.ftc.teamcode.robotSubSystems.Elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.ShooterBlueBalls.ShooterBlueBalls;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.ShooterGreenBalls.ShooterGreenBalls;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.TankGrabber.TankGrabber;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;

@TeleOp(name = "main")
public class Robot extends LinearOpMode {
    // * set new robot pose to 0,0 and heading to 0

    ElapsedTime currentTime = new ElapsedTime();
    Shooter shooterBlueBalls = new ShooterBlueBalls();
    Shooter shooterGreenBalls = new ShooterGreenBalls();

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        Conveyor.init(hardwareMap);
        Elevator.init(hardwareMap);
        Intake.init(hardwareMap);
        shooterBlueBalls.init(hardwareMap);
        shooterGreenBalls.init(hardwareMap);
        TankGrabber.init(hardwareMap);


//        OrbitGyro.init(this.hardwareMap);

        GlobalData.voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GlobalData.currentVoltage = GlobalData.voltageSensor.getVoltage();

        currentTime.reset();

        waitForStart();

        while (!isStopRequested()){
            GlobalData.currentTime = (float) currentTime.milliseconds();
            GlobalData.currentVoltage = GlobalData.voltageSensor.getVoltage();

            telemetry.addData("servo", Elevator.elevatorServo.getPosition());
            SubSystemManager.setSubsystemToState(SubSystemManager.getStateFromJoystick(gamepad1), gamepad1, telemetry);

            GlobalData.currentRobotState = SubSystemManager.getStateFromJoystick(gamepad1);
            telemetry.update();
        }

    }

}