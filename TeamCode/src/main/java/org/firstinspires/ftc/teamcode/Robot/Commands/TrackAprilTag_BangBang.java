package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.aprilTagBangBangTeleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TrackAprilTag_BangBang extends Command {
    Gamepad gamepad;
    public TrackAprilTag_BangBang(Gamepad gamepad){
        this.gamepad = gamepad;
    }
    @Override
    public void start() {

    }
    @Override
    public void update() {
        aprilTagBangBangTeleop(gamepad);
    }


    @Override
    public boolean isDone() {
        return false;
    }
}
