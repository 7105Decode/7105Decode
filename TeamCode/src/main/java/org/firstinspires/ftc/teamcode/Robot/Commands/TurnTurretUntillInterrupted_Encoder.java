package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TurnTurretUntillInterrupted_Encoder extends Command {
    double targetposition,timeout;
    Turret turret;
    ElapsedTime timer = new ElapsedTime();
    public TurnTurretUntillInterrupted_Encoder(Turret turret,double targetposition, double timeout){
        this.targetposition = targetposition;
        this.turret = turret;
        this.timeout = timeout;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
        turret.turret.setPower(turret.pController.calculate(turret.getCurrentPosition(),targetposition));
    }

    @Override
    public boolean isDone() {
        return Math.abs(turret.getError(targetposition)) <= 8 || timer.seconds() >= timeout;
    }

    @Override
    public void stop(boolean interrupted) {
        turret.turret.setPower(0);
    }
}
