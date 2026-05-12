package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TurnTurret_Encoder extends Command {
    Turret turret;
    ElapsedTime timer = new ElapsedTime();
    double time, target;
    public TurnTurret_Encoder(Turret turret, double time, double target){
        this.time=time;
        this.turret=turret;
        this.target=target;
    }

    @Override
    public void start() {
        timer.reset();
        turret.pController.reset();
    }

    @Override
    public void update() {
        turret.pController.calculate(turret.getCurrentPosition(),target);
    }

    @Override
    public boolean isDone() {
        return timer.seconds()>=time || turret.getError(target)>= 8;
    }
}
