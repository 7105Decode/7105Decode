package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TrackAprilTag_BangBang extends Command {
    Limelight3A limelight;
    Subsystem subsystem;
    public TrackAprilTag_BangBang(Limelight3A limelight, Turret turret){
        this.limelight = limelight;
        this.subsystem = turret;
    }
    @Override
    public void start() {

    }
    @Override
    public void update() {

    }


    @Override
    public boolean isDone() {
        return false;
    }
}
