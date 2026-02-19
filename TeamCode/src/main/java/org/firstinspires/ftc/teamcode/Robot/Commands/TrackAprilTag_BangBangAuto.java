package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TrackAprilTag_BangBangAuto extends Command {
    int pipeline;
    public TrackAprilTag_BangBangAuto(int pipeline){
        this.pipeline = pipeline;
    }
    @Override
    public void start() {
        Turret.INSTANCE.limelight.pipelineSwitch(pipeline);
        Turret.INSTANCE.limelight.start();
        Turret.doneTrackingAprilTagAuto = false;
    }
    @Override
    public void update() {
        Turret.INSTANCE.result = Turret.INSTANCE.limelight.getLatestResult();
        Turret.INSTANCE.aprilTagBangBang_Auto();
    }


    @Override
    public boolean isDone() {
        return Turret.doneTrackingAprilTagAuto;
    }
}
