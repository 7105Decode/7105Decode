package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public static PathChain RedShortPreloads, RedShortCollectPPG, RedShortScorePPG, RedShortCollectPGP, RedShortScorePGP, RedShortCollectGPP;

    public static Pose RedShortStartPose = new Pose(121.6, 124.5,Math.toRadians(216.5));

    public Paths(Follower follower) {
        RedShortPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                RedShortStartPose,

                                new Pose(109.9, 108.7)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(216.5))
                .build();

        RedShortCollectPPG = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(110, 108.7),
                                new Pose(83, 89.7),
                                new Pose(111.3, 83.5),
                                new Pose(125.4, 82.6)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        RedShortScorePPG = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.4, 82.6),

                                new Pose(103.9, 97)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))

                .build();
        RedShortCollectPGP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(103.9, 97),
                                new Pose(86.7, 69.6),
                                new Pose(101, 63.6),
                                new Pose(126, 59)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        RedShortScorePGP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126, 59),

                                new Pose(92, 85)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))

                .build();

        RedShortCollectGPP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92, 85),
                                new Pose(89.7, 49),
                                new Pose(92.4, 40),
                                new Pose(126, 34.4)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}

