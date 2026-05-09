package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class FishForAprilTagBlueAuto extends Command {
    int pipeline;
    boolean turnRight;
    ElapsedTime timer = new ElapsedTime();
    double power, offset, time;
    public FishForAprilTagBlueAuto(int pipeline, boolean turnRight, double power,double offset, double time){
        this.pipeline = pipeline;
        this.time = time;
        this.offset = offset;
        this.power = power;
        this.turnRight = turnRight;
    }
    @Override
    public void start() {
        Turret.INSTANCE.limelight.pipelineSwitch(pipeline);
        if (!Turret.turnLimelightOn) {
            Turret.INSTANCE.limelight.start();
            Turret.turnLimelightOn = true;
        }
        timer.reset();
        Turret.doneTrackingAprilTagAuto = false;
    }
    @Override
    public void update() {
        Turret.INSTANCE.result = Turret.INSTANCE.limelight.getLatestResult();
        Turret.INSTANCE.fishingForAprilTag_BlueAuto(turnRight,power,offset);
    }


    @Override
    public boolean isDone() {
        return Turret.doneTrackingAprilTagAuto|| timer.seconds() > time;
    }

    @Override
    public void stop(boolean interrupted) {
        Turret.doneTrackingAprilTagAuto = false;
    }
}
