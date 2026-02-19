package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooterNoController extends Command {
    @Override
    public void start() {
        Shooter.runShooter = false;
    }
    @Override
    public boolean isDone() {
        return true;
    }
}