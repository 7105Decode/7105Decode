package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot.Commands.RunRightTransferServo;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

@TeleOp
@Configurable
@Disabled
public class CommandOpmode extends CommandOpMode {
    // in your implementation of CommandOpMode
    Robot robot;
    GamepadEx firstController, secondController;
    Button exampleButton;
    @Override
    public void initialize() {
        robot = new Robot(Robot.OpModeType.TELEOP,hardwareMap,new Pose(0,0,0),telemetry);
        firstController = new GamepadEx(gamepad1);
        secondController = new GamepadEx(gamepad2);
         exampleButton = new GamepadButton(
                firstController, GamepadKeys.Button.A
        );
        // schedule all commands
//        schedule(new RunRightTransferServo(robot.transfer, Transfer.RightTransferStates.TRANSFER));
        register(robot.driveTrain,robot.shooter,robot.dashboard);
    }

    @Override
    public void initialize_loop() {
        robot.initLoopTele();
    }

    @Override
    public void run() {
        super.run();
//        exampleButton.whenPressed(new RunRightTransferServo(robot.transfer, Transfer.RightTransferStates.TRANSFER));
        robot.updateRobotRun();
    }
}
