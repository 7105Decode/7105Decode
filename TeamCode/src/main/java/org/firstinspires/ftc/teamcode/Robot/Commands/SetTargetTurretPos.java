package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class SetTargetTurretPos extends Command {
    double targetPos;
    public SetTargetTurretPos(double targetPos){
        this.targetPos =targetPos;
    }
    @Override
    public void start() {
        Turret.targetPos = targetPos;
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
