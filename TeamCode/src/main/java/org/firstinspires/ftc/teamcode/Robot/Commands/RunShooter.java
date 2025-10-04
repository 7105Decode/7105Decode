package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class RunShooter extends InstantCommand {
    public RunShooter(Shooter.ShooterStates state) {
//        super(
//                () -> RobotHardware.getInstance().intake.updateState(state)
//        );
    }
}