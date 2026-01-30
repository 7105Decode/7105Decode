package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.intakeSpeed;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.runIntakeTele;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import java.util.Set;
public class RunIntakeTele extends Command {
    public boolean interruptible = false;
    Gamepad gamepad1;
    public RunIntakeTele(Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        interruptible = false;
    }
    @Override
    public boolean isDone() {
        return false; // Whether or not the command is done
    }
    @Override
    public void start() {

    }
    @Override
    public void update() {
        runIntakeTele(gamepad1);
    }

    @Override
    public void stop(boolean interrupted) {

    }
}