package org.firstinspires.ftc.teamcode.Robot.Commands;

//import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class FollowPath extends Command {
    public boolean interruptible, done;
    PathChain path;
    public FollowPath(PathChain path) {
//        this.subsystem = subsystem;
        this.path = path;
        interruptible = false;
    }
    @Override
    public boolean isDone() {
        return !DriveTrain.INSTANCE.follower.isBusy(); // Whether or not the command is done
    }
    @Override
    public void start() {
        DriveTrain.INSTANCE.followPath(path,true);
    }
    @Override
    public void update() {

    }

    @Override
    public void stop(boolean interrupted) {
    }
}