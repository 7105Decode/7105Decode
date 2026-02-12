package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class AprilTagGuidedWithEncoder extends Command {
    Subsystem subsystem;
    double thresholdpos;

    public AprilTagGuidedWithEncoder(Subsystem subsystem, double turretpower,double thresholdpos){
        this.thresholdpos = thresholdpos;
        this.subsystem = subsystem;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isDone() {
        return Turret.getCurrentPosition() >= thresholdpos;
    }
}