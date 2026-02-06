package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class MoveTurret extends Command {
    double targetPos;
    public MoveTurret(double targetPos){
        this.targetPos =targetPos;
    }
    @Override
    public void start() {
        Turret.targetPos = targetPos;
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
