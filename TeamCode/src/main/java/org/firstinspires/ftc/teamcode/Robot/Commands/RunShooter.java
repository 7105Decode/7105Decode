package org.firstinspires.ftc.teamcode.Robot.Commands;


import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooter extends CommandBase {
    Shooter shooter;
    Shooter.ShooterStates shooterStates;
    public RunShooter(Shooter shooter, Shooter.ShooterStates shooterStates){
        this.shooter = shooter;
        this.shooterStates = shooterStates;
    }

    @Override
    public void initialize() {
        shooter.setShooterStates(shooterStates);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
