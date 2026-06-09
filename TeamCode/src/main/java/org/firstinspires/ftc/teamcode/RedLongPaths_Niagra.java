package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedLongPaths_Niagra {
        public PathChain GrabCorner, BackUp,PrepShift,ShiftLastArtifact,Park,GrabLastArtifact, ReturnToShoot,GrabLine,Shoot;
        public static Pose startpose = new Pose(85.8, 8.7,Math.toRadians(0));
        public RedLongPaths_Niagra(Follower follower) {

            GrabCorner = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startpose,

                                    new Pose(135.5, 14.8)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            BackUp = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.5, 14.8),

                                    new Pose(125.46, 12.26)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            PrepShift = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.460, 12.26),

                                    new Pose(132.7, 10.67)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShiftLastArtifact = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.67, 10.7),

                                    new Pose(132.6, 10.4)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))

                    .build();

            GrabLastArtifact = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.6, 10.4),

                                    new Pose(135.5, 10.2)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            ReturnToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(140.5, 10.2),

                                    startpose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            GrabLine = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    startpose,
                                    new Pose(90.1, 45.2),
                                    new Pose(126.9, 39.9)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.8, 34),
                                    new Pose(85.8, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))

                    .build();
            Shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.9, 39.9),

                                    new Pose(85.8, 8.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }

}
