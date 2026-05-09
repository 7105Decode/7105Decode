package org.firstinspires.ftc.teamcode.Robot.Commands;
//import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class FollowPathTimer extends Command {
    public boolean interruptible;
    PathChain path;
    Subsystem subsystem;
    ElapsedTime timer = new ElapsedTime();
    double time;
    public FollowPathTimer(Subsystem subsystem,PathChain path, double time) {
        this.subsystem = subsystem;
        this.path = path;
        interruptible = true;
        this.time = time;
    }
    @Override
    public void start() {
        DriveTrain.INSTANCE.followPath(path,true);
        DriveTrain.updateDriveTrain = true;
        timer.reset();
    }
    @Override
    public boolean isDone() {
        return !DriveTrain.INSTANCE.follower.isBusy() || timer.seconds() >= time; // Whether or not the command is done
    }
    @Override
    public void stop(boolean interrupted) {
//        DriveTrain.updateDriveTrain = false;
    }
}