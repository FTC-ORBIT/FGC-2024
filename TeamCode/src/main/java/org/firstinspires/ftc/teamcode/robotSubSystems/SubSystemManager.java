package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.Conveyor.Conveyor;
import org.firstinspires.ftc.teamcode.robotSubSystems.Conveyor.ConveyorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.Elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.ShooterBlueBalls.ShooterBlueBalls;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.ShooterGreenBalls.ShooterGreenBalls;
import org.firstinspires.ftc.teamcode.robotSubSystems.Shooter.ShooterState;
import org.firstinspires.ftc.teamcode.robotSubSystems.TankGrabber.TankGrabber;
import org.firstinspires.ftc.teamcode.robotSubSystems.TankGrabber.TankGrabberStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class SubSystemManager {

   public static ConveyorState conveyorState = ConveyorState.STOP;
   private static IntakeState intakeState = IntakeState.STOP;
   private static ElevatorState elevatorState = ElevatorState.STOP;
   private static TankGrabberStates tankGrabberState = TankGrabberStates.OPEN;
   private static ShooterState shooterBlueBallsState = ShooterState.STOP;
   private static ShooterState shooterGreenBallsState = ShooterState.STOP;
   private static boolean lastRightBumper;
   private static Shooter shooterBlueBalls = new ShooterBlueBalls();
   private static Shooter shooterGreenBalls = new ShooterGreenBalls();
    private static RobotState prevRobotState = GlobalData.robotState;



    public static RobotState getStateFromJoystick(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                : gamepad.x ? RobotState.SHOOT_BLUE
                : gamepad.y ? RobotState.SHOOT_GREEN : prevRobotState;
    }

    public static void setSubsystemToState(final RobotState robotState, Gamepad gamepad, Telemetry telemetry) {
        switch (robotState) {
            case TRAVEL:
            default:
                intakeState = IntakeState.STOP;
                shooterBlueBallsState = ShooterState.STOP;
                shooterGreenBallsState = ShooterState.STOP;
                conveyorState = ConveyorState.STOP;
                break;
            case INTAKE:
                conveyorState = ConveyorState.INTAKE;
                intakeState = IntakeState.INTAKE;
                break;
            case SHOOT_BLUE:
                conveyorState = ConveyorState.INTAKE;
                intakeState = IntakeState.INTAKE;
                shooterBlueBallsState = ShooterState.SHOOT;
                break;
            case SHOOT_GREEN:
                conveyorState = ConveyorState.TRANSPORT;
                intakeState = IntakeState.SHOOTER_GREEN;
                shooterGreenBallsState = ShooterState.SHOOT;
                break;
        }

        if (!lastRightBumper && gamepad.right_bumper){
            tankGrabberState = tankGrabberState.equals(TankGrabberStates.OPEN) ? TankGrabberStates.CLOSED : TankGrabberStates.OPEN;
        }

        if (Math.abs(gamepad.right_stick_y) > 0.3) {
            intakeState = IntakeState.OVERRIDE;
            telemetry.addData("over 0.2", null);
        } else if (Math.abs(gamepad.right_stick_x) > 0.2){
            conveyorState = ConveyorState.OVERRIDE;
        }

        if (gamepad.right_bumper){ // I'm aware that this is not the best way to handle this situation, yet, I didn't find a better solution and due to the lack of time i'm not sure it's worth finding the best solution
            elevatorState = ElevatorState.CLIMB;
        } else if (gamepad.left_bumper){
            elevatorState = ElevatorState.CLOSED;
        } else if (gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_right){
            elevatorState = ElevatorState.OVERRIDE;
        }

        Drivetrain.operate(-gamepad.left_stick_y, gamepad.right_trigger, gamepad.left_trigger, telemetry, gamepad);
        Elevator.operate(elevatorState, gamepad,telemetry);
        Intake.operate(intakeState, gamepad);
        shooterBlueBalls.operate(shooterBlueBallsState, gamepad);
        shooterGreenBalls.operate(shooterGreenBallsState, gamepad);
        TankGrabber.operate(tankGrabberState);
        Conveyor.operate(conveyorState, gamepad);


        telemetry.addData("elevatorState", elevatorState);
        telemetry.update();
        prevRobotState = robotState;
    }
}
