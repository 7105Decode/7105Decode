package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
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
robot= new Robot(Robot.OpModeType.TELEOP,hardwareMap,new Pose());

        gamepad= new GamepadEx(gamepad1);
        gamepad_2= new GamepadEx(gamepad2);
        CommandScheduler.getInstance().enable();
//        robot.updateRobotInit();
    }

    @Override
    public void initialize_loop() {

        schedule(gamepad.isDown(GamepadKeys.Button.A)));
        register(robot.shooter, robot.driveTrain, robot.dashboard);
    }

    @Override
    public void run() {
       super.run();
        robot.updateRobotRun();

    }
}
