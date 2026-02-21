package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class FishForAprilTagRedAuto extends Command {
    int pipeline;
    boolean turnRight;
    double power, offset;
    public FishForAprilTagRedAuto(int pipeline, boolean turnRight, double power,double offset){
        this.pipeline = pipeline;
        this.offset = offset;
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
        Turret.INSTANCE.result = Turret.INSTANCE.limelight.getLatestResult();
        Turret.INSTANCE.fishingForAprilTag_RedAuto(turnRight,power,offset);
    }


    @Override
    public boolean isDone() {
        return Turret.doneTrackingAprilTagAuto;
    }
}
