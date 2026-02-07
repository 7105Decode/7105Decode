package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

public class RunIntakeAuto extends Command {
    boolean runIntakeAuto;
    public RunIntakeAuto(boolean runIntakeAuto){
        this.runIntakeAuto = runIntakeAuto;
    }
    @Override
    public void start() {
        Intake.runIntakeAuto = runIntakeAuto;
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
