package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class MoveHood extends Command {
    Shooter shooter;
    public MoveHood(Shooter shooter){
        this.shooter = shooter;
    }
    @Override
    public void start() {
//        shooter.set
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
