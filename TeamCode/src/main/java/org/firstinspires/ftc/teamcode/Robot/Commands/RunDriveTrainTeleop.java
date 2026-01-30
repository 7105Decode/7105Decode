package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class RunDriveTrainTeleop extends Command {
    DriveTrain driveTrain;
    Gamepad gamepad1;
    public RunDriveTrainTeleop(DriveTrain driveTrain,Gamepad gamepad1){
        this.driveTrain = driveTrain;
        this.gamepad1 = gamepad1;
    }

    @Override
    public void start() {
        driveTrain.fieldCentricDrive(gamepad1);
    }

    @Override
    public void update() {
        driveTrain.fieldCentricDrive(gamepad1);
    }

    @Override
    public boolean isDone() {
        return false;
    }
}
