package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

public class TurnIntakeOff extends Command {
    @Override
    public void start() {
        Intake.runIntakeAuto = false;
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
