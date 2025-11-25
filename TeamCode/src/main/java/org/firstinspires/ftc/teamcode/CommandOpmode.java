package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

@TeleOp
public class CommandOpmode extends CommandOpMode {
    Robot robot;
    GamepadEx gamepad,gamepad_2;
    @Override
    public void initialize() {
        robot = new Robot(hardwareMap,telemetry,new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0));

        gamepad= new GamepadEx(gamepad1);
        gamepad_2= new GamepadEx(gamepad2);

        robot.dashboard.updateTelemetry();
    }

    @Override
    public void run() {
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenHeld(new RunShooter(robot.shooter, Shooter.ShooterStates.MAXSPEED))
                .whenReleased(new RunShooter(robot.shooter, Shooter.ShooterStates.STOP));
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenHeld(new RunShooter(robot.shooter, Shooter.ShooterStates.HALFSPEED))
                .whenReleased(new RunShooter(robot.shooter, Shooter.ShooterStates.STOP));
        robot.robotTelem();
        robot.dashboard.updateTelemetry();
    }
}
