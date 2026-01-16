package org.firstinspires.ftc.teamcode.Robot.Commands;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;

public class FollowPath_PP extends CommandBase {
    Follower follower;
    Path path;
    double headingtolerance;
    public FollowPath_PP(Follower follower,Path path, double headingtolerance){
        this.follower = follower;
        this.path = path;
        this.headingtolerance = headingtolerance;
    }

    @Override
    public void initialize() {
        follower.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
