package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.pedropathing.paths.Path;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class RunShooter extends Command {
    private final Subsystem subsystem;
    public boolean interruptible;
    public RunShooter(Subsystem subsystem) {
        this.subsystem = subsystem;
        interruptible = false;
    }
    @Override
    public boolean isDone() {
        return false;
    }
    @Override
    public void start() {
    }
    @Override
    public void update() {

    }

    @Override
    public void stop(boolean interrupted) {

    }
}