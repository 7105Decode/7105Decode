package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooter extends Command {
    Shooter shooter;
    double vel, kp;
    public RunShooter(Shooter shooter, double kp, double vel) {
        this.shooter = shooter;
        this.vel = vel;
        this.kp = kp;
    }
    @Override
    public boolean isDone() {
        return true;
    }
    @Override
    public void start() {
        shooter.setControllerValue(kp);
        shooter.setTargetVel(vel);
        Shooter.turnShooterOff = true;
    }
    @Override




    public void update() {

    }

    @Override
    public void stop(boolean interrupted) {

    }
}