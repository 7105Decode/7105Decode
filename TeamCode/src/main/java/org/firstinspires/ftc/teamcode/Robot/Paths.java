package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public static PathChain RedShortPath1, RedShortPath2, RedShortPath3;

    public static Pose RedShortStartPose = new Pose(124.6, 120,Math.toRadians(216.5));

    public Paths(Follower follower) {
        RedShortPath1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                RedShortStartPose,

                                new Pose(109.9, 108.7)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(216.5))

                .build();

        RedShortPath2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(110, 108.7),
                                new Pose(83, 89.7),
                                new Pose(111.3, 83.5),
                                new Pose(125.4, 82.6)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        RedShortPath3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.4, 82.6),

                                new Pose(105.9, 94)
                        )
                ).setConstantHeadingInterpolation(Math.toDegrees(0))

                .build();
    }
}