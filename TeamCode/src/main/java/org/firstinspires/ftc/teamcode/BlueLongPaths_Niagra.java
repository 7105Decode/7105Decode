package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueLongPaths_Niagra {
        public PathChain Path1,Path2,Path3,Path4,Path5,Path6,Path7, Path8, Path9, Path10;
        //Only adjust startpose if necessary. I wouldn't mess with this though.
        public static Pose startpose =new Pose(58.2, 8.7,Math.toRadians(180));
        public BlueLongPaths_Niagra(Follower follower) {
            //Path1 grabs off the line.
            Path1 = follower.pathBuilder().addPath(
                    // Below the first pos is where the robot is starting from. The ast pose so new Pose(17.1, 34.95) is the pose we are driving to
                    //the middle pose, so new Pose(48.8, 41.2), is a control point. You shouldn't need to adjust the
                            new BezierCurve(startpose,
                                    new Pose(50.8, 42.2),
                                    new Pose(17.1, 37)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            //Path2 goes back to shoot.
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.1, 34.950),
                                    new Pose(45, 8.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(190))
                    .build();

            //Path3 goes to corner to collect artifacts.
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.2, 8.7),
                                    new Pose(8.5, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            //This is backing up just a little.
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.5, 11.8),
                                    new Pose(18.5, 12.2)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            //This should turn the robot to get the last artifact in the corner.
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.5, 12.2),
                                    new Pose(9.3, 8)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            //Get the artifact out
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.3, 8),
                                    new Pose(10, 9)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(155))
                    .build();

            //Return to shoot
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 9),
                                    new Pose(45, 8.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            //Pickup from corner. Fishing for more artifacts.
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.2, 8.7),
                                    new Pose(8.8, 14)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            //Return to shoot.
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.5, 14),
                                    new Pose(45, 8.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            //Park
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45, 8.7),
                                    new Pose(40, 23)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                    .build();
        }
}
