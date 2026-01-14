package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath_PP;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class BlueShortAuto extends CommandOpmode {
    Robot robot;
    FollowPath_PP firstpath,secondpath,thirdpath;
    @Override
    public void initialize() {
        robot = new Robot(Robot.OpModeType.AUTO,hardwareMap,new Pose(56,8,180),telemetry);
        firstpath = new FollowPath_PP(new Path(),hardwareMap,4);
//        secondpath = new FollowPath_PP(new);
//        thirdpath = new FollowPath_PP();

        register(robot.driveTrain,robot.shooter,robot.dashboard);
    }

    @Override
    public void initialize_loop() {
        robot.initLoopAuto();
    }

    @Override
    public void run() {
        super.run();
        robot.updateRobotRun();
    }
}
