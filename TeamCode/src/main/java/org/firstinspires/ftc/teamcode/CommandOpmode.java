package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

@TeleOp
@Configurable
public class CommandOpmode extends CommandOpMode {
    Robot robot;
    GamepadEx gamepad,gamepad_2;
    @Override
    public void initialize() {
robot= new Robot(hardwareMap,new Pose());

        gamepad= new GamepadEx(gamepad1);
        gamepad_2= new GamepadEx(gamepad2);
        robot.updateRobotInit();
    }

    @Override
    public void initialize_loop() {
        robot.updateRobotInit();
    }

    @Override
    public void run() {
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenHeld(new RunShooter(robot.shooter, Shooter.ShooterStates.MAXSPEED))
                .whenReleased(new RunShooter(robot.shooter, Shooter.ShooterStates.STOP));
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenHeld(new RunShooter(robot.shooter, Shooter.ShooterStates.HALFSPEED))
                .whenReleased(new RunShooter(robot.shooter, Shooter.ShooterStates.STOP));
        robot.updateRobotRun();
    }
}
