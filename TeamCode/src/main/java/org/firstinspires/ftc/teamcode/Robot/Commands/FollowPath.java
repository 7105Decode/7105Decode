package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.pedropathing.paths.Path;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class FollowPath extends Command {
    private final Subsystem subsystem;
    public boolean interruptible, done;
    Path path;
    public FollowPath(Subsystem subsystem, Path path) {
        this.subsystem = subsystem;
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
    }
    @Override
    public void update() {
        if (!DriveTrain.follower.isBusy()){
            done = true;
        }
        DriveTrain.follower.followPath(path,true);
    }

    @Override
    public void stop(boolean interrupted) {
    }
}