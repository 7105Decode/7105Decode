package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueLongPaths {
    public PathChain GrabCorner,NinePark, BackUp,PrepShift,ShiftLastArtifact,GrabLastArtifact, ReturnToShoot,GrabLine,ShootAgain,
            SafePark;
    public static Pose BlueLongStartPose = new Pose(58.2, 8.7,Math.toRadians(180));
    public BlueLongPaths(Follower follower) {
        GrabCorner = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.2, 8.7),

                                new Pose(7.5, 11.8)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BackUp = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.5, 11.8),

                                new Pose(18.5, 12.2)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PrepShift = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.5, 12.2),

                                new Pose(9.3, 8)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        ShiftLastArtifact = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.3, 8),

                                new Pose(10, 9)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(155))
                .build();

        GrabLastArtifact = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10, 7),
                                new Pose(8, 7)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ReturnToShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8, 7),
                                new Pose(43.7, 8.7)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        GrabLine = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(43.7, 8.7),
                                new Pose(48.8, 41.2),
                                new Pose(17.1, 34.95)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ShootAgain = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.107, 34.954),

                                new Pose(43.7, 8.7)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        NinePark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.7, 8.7),

                                new Pose(23, 8.7)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        SafePark = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8, 7),
                                new Pose(24.7, 25.8),
                                new Pose(30.5, 10.1)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }
}