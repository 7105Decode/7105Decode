package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class RunDriveTrain extends CommandBase {
    DriveTrain driveTrain;
    Gamepad gamepad;
    public RunDriveTrain(DriveTrain driveTrain, Gamepad gamepad){
        this.driveTrain = driveTrain;
        this.gamepad = gamepad;
    }

    @Override
    public void initialize() {
        driveTrain.fieldCentricDrive(gamepad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
