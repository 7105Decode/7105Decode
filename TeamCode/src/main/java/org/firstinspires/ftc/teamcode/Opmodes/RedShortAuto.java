package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath_PP;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class RedShortAuto extends CommandOpmode {
    Robot robot;
    FollowPath_PP firstpath,secondpath,thirdpath;
    @Override
    public void initialize() {
        robot = new Robot(Robot.OpModeType.AUTO,hardwareMap,new Pose(0,0,0),telemetry);
//        firstpath = new FollowPath_PP();
//        secondpath = new FollowPath_PP();
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
