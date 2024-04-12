package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.MaxHeightForFourbarDelay;
import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;
import static org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni.motors;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveByAprilTags.Camera;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Sensors.MagneticSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistanceSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.Sensors.Potentiometer;
import org.firstinspires.ftc.teamcode.Sensors.TouchSensor;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.openftc.apriltag.AprilTagDetection;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;

    public static RobotState wanted = RobotState.TRAVEL;



    private static RobotState getState(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                : gamepad.x ? RobotState.MIN : gamepad.y ? RobotState.LOW :gamepad.start ? RobotState.DEPLETE: gamepad.dpad_up ? RobotState.FIXPIXEL : gamepad.right_bumper ? RobotState.MID: lastState;
    }

    private static RobotState getStateFromWantedAndCurrent(RobotState stateFromDriver) {

        switch (stateFromDriver) {
            case INTAKE:
                break;
            case LOW:
                break;
            case MID:
                break;
            case HIGH:
                break;
            case TRAVEL:
                break;
            case DEPLETE:
                break;
            case FIXPIXEL:
                break;
            case MIN:
                break;

        }
        return stateFromDriver;
    }

    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
//        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));
        wanted = getState(gamepad1);




        switch (wanted) {
            case TRAVEL:
                break;
            case INTAKE:
                break;
            case LOW:
                break;
            case HIGH:
                break;
            case DEPLETE:
                break;
            case FIXPIXEL:
                break;
            case MIN:
                break;
            case MID:
                break;
        }

        lastState = wanted;
        if (gamepad1.dpad_down) OrbitGyro.resetGyro();
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("Robot current state ", SubSystemManager.wanted);
        telemetry.addData("Robot last state", SubSystemManager.lastState);
        if (Camera.targetFound){
            telemetry.addData("tag has found",Camera.desiredTag.id);
        }else if (Camera.skippingTagTelemetry){
            telemetry.addData("other tag ha found, skipping",Camera.desiredTag.id);
        }else {
            telemetry.addLine("tag not in the lib");
        }
        telemetry.addData("gyro", Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.addData("currentTime", GlobalData.currentTime);
        telemetry.addData("lastTime", GlobalData.lastTime);
        telemetry.addData("deltaTime",GlobalData.deltaTime);
        telemetry.addData("lf power" , motors[0].getPower());
        telemetry.addData("rf power" , motors[1].getPower());
        telemetry.addData("lb power" , motors[2].getPower());
        telemetry.addData("rb power" , motors[3].getPower());
//        telemetry.addData("distance in inch", OrbitDistanceSensor.getDistance());
//        telemetry.addData("color", OrbitColorSensor.hasGamePiece());
//        telemetry.addData("magnetic press?", MagneticSensor.getState());
//        telemetry.addData("touchSensor press?", TouchSensor.getState());
//        telemetry.addData("potentiometer", Potentiometer.getVolt());
    }
}

//dani yalechan!
// yoel yalechan!