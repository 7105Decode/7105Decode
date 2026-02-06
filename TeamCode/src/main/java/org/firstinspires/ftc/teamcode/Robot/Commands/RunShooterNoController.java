package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.pedropathing.paths.Path;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooterNoController extends Command {
    Shooter shooter;
    double power;
    public RunShooterNoController(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
    }
    @Override
    public boolean isDone() {
        return false;
    }
    @Override
    public void start() {
        shooter.setPower(power);
    }
    @Override
    public void update() {
        shooter.setPower(power);
    }

    @Override
    public void stop(boolean interrupted) {

    }
}