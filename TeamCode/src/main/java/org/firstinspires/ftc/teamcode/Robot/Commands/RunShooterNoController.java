package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooterNoController extends Command {
//    Shooter shooter;
//    double power;
    public RunShooterNoController() {
//        this.shooter = shooter;
//        this.power = power;
    }
    @Override
    public boolean isDone() {
        return true;
    }
    @Override
    public void start() {
        Shooter.turnShooterOff = false;
    }
}