package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

public class FollowPathBuilder_PP extends CommandBase {
    Follower follower;
    Path path;

    double headingtolerance;
    public FollowPathBuilder_PP(Path path, HardwareMap hardwareMap, double headingtolerance){
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
