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
        return done; // Whether or not the command is done
    }
    @Override
    public void start() {
        done = false;
        DriveTrain.INSTANCE.followPath(path,true);
    }
    @Override
    public void update() {
        if (!DriveTrain.INSTANCE.follower.isBusy()){
            done = true;
        }
    }

    @Override
    public void stop(boolean interrupted) {
    }
}