package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.intakeSpeed;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

import java.util.Set;

public class RunIntakeTele extends Command {

    private final Set<Subsystem> subsystems;
    private final boolean interruptible = true;
    Gamepad gamepad1;
    public RunIntakeTele(Set<Subsystem> subsystems, Gamepad gamepad1) {
        this.subsystems = subsystems;
        this.gamepad1 = gamepad1;
    }
    @Override
    public boolean isDone() {
        return false; // Whether or not the command is done
    }
    @Override
    public void start() {

    }
    @Override
    public void update() {
        if (gamepad1.left_trigger > .4){
            Intake.INSTANCE.intake.setPower(-intakeSpeed);
        } else if (gamepad1.right_trigger > .4) {
            Intake.INSTANCE.intake.setPower(intakeSpeed);
        } else {
            Intake.INSTANCE.intake.setPower(0);
        }
    }

    @Override
    public void stop(boolean interrupted) {

    }
}