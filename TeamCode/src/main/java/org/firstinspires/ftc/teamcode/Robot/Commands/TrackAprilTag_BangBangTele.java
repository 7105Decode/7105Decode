package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TrackAprilTag_BangBangTele extends Command {
    Gamepad gamepad;
    public TrackAprilTag_BangBangTele(Gamepad gamepad){
        this.gamepad = gamepad;
    }
    @Override
    public void start() {

    }
    @Override
    public void update() {
        Turret.INSTANCE.result = Turret.INSTANCE.limelight.getLatestResult();
        Turret.INSTANCE.aprilTagBangBang_Teleop(gamepad);
    }


    @Override
    public boolean isDone() {
        return false;
    }
}
