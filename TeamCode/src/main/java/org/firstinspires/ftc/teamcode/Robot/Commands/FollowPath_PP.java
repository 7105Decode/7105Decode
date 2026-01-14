package org.firstinspires.ftc.teamcode.Robot.Commands;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;

public class FollowPath_PP extends CommandBase {
    Follower follower;
    Path path;

    double headingtolerance;
    public FollowPath_PP(Path path, HardwareMap hardwareMap, double headingtolerance){
        this.path = path;
        this.headingtolerance = headingtolerance;
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void initialize() {
        follower.followPath(path,true);
    }

    @Override
    public boolean isFinished() {
        return follower.atParametricEnd() && follower.getHeadingError() < headingtolerance;
    }
}
