package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class FishForAprilTagAuto extends Command {
    int pipeline;
    boolean turnRight;
    double power;
    public FishForAprilTagAuto(int pipeline,boolean turnRight,double power){
        this.pipeline = pipeline;
        this.power = power;
        this.turnRight = turnRight;
    }
    @Override
    public void start() {
        Turret.INSTANCE.limelight.pipelineSwitch(pipeline);
//        if (Turret.INSTANCE.limelight.getStatus() != Turret.INSTANCE.limelight.) {
            Turret.INSTANCE.limelight.start();
//        }
        Turret.doneTrackingAprilTagAuto = false;
    }
    @Override
    public void update() {
        Turret.INSTANCE.fishingForAprilTag_BangBang_Auto(turnRight,power);
    }


    @Override
    public boolean isDone() {
        return Turret.doneTrackingAprilTagAuto;
    }
}
