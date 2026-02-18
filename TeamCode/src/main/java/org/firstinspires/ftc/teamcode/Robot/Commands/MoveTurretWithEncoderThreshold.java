package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class MoveTurretWithEncoderThreshold extends Command {
    Subsystem subsystem;
    double thresholdpos;
    boolean turnRight;
    double targetPos;
    public MoveTurretWithEncoderThreshold(Subsystem subsystem, boolean turnRight, double targetPos,double thresholdpos){
        this.thresholdpos = thresholdpos;
        this.subsystem = subsystem;
        this.turnRight = turnRight;
        this.targetPos = targetPos;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        if (turnRight){
            Turret.INSTANCE.turret.setPower(.55);
        } else {
            Turret.INSTANCE.turret.setPower(-.55);
        }
    }

    @Override
    public boolean isDone() {
        return Math.abs(Turret.INSTANCE.getError(targetPos)) <= thresholdpos;
    }

    @Override
    public void stop(boolean interrupted) {
        Turret.INSTANCE.turret.setPower(0);
    }
}