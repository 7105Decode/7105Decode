package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

@Autonomous(name = "NextFTC + Pedro: FollowPath (Java)")
public class AutoFollowPath extends PedroOpMode /* (Claw, Lift) */ {

    private final Pose startPose = new Pose(9.0, 60.0, Math.toRadians(0.0));
    private final Pose finishPose = new Pose(37.0, 50.0, Math.toRadians(180.0));

    private PathChain move;

    public void buildPaths() {
        move = follower.pathBuilder()
                .addPath(new BezierLine(startPose,finishPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        follower.setPose(startPose);
        buildPaths();

        SequentialGroup routine = new SequentialGroup(
                new FollowPath(move)
                // more commands...
        );

//        schedule(routine);
    }
}