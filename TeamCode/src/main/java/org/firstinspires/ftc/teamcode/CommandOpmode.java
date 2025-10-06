package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Commands.RunDriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
public class CommandOpmode extends CommandOpMode {
    Robot robot;
    Gamepad gamepad,gamepad_2;
    @Override
    public void initialize() {
        robot= new Robot(hardwareMap,telemetry,new Pose2d(0,0,0));
        gamepad= new Gamepad();
        gamepad_2= new Gamepad();
    }

    @Override
    public void run() {
        schedule(new RunCommand(telemetry::update),new RunDriveTrain(robot.driveTrain,gamepad));
    }
}
