package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooter extends Command {
    double vel,feedforward;
    public RunShooter(double feedforward, double vel) {

        this.feedforward = feedforward;
        this.vel = vel;
    }
    @Override
    public boolean isDone() {
        return Shooter.runShooter;
    }
    @Override
    public void start() {
        Shooter.runShooter = true;
        Shooter.INSTANCE.setTargetVel(vel);
        Shooter.INSTANCE.setTargetFeedForward(feedforward);
    }
    @Override
    public void stop(boolean interrupted) {
        Shooter.INSTANCE.setPower(0);
    }
}