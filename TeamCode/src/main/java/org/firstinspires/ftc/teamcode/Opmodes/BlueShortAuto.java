package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.field.Line;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath_PP;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous
public class BlueShortAuto extends CommandOpMode {
    Robot robot;
    FollowPath_PP firstpath,secondpath,thirdpath;
    Pose startpose, firstpose,secondpose;
    @Override
    public void initialize() {
        startpose = new Pose(31.4,135.3,180);
        firstpose = new Pose(35.3, 132.9,180);
        secondpose = new Pose(60, 132.7,270);
        robot = new Robot(Robot.OpModeType.AUTO,hardwareMap,startpose,telemetry);
        firstpath = new FollowPath_PP(new Path(new BezierLine(startpose,firstpose)),hardwareMap,4);
        secondpath = new FollowPath_PP(new Path(new BezierLine(firstpose,secondpose)),hardwareMap,4);
//        thirdpath = new FollowPath_PP();

        register(robot.driveTrain,robot.shooter,robot.dashboard,robot.transfer);
    }
    @Override
    public void initialize_loop() {
        robot.driveTrain.follower.update();
        robot.initLoopAuto();
    }
    @Override
    public void waitForStart() {
        super.waitForStart();
//        schedule(firstpath.andThen(secondpath));
        robot.driveTrain.follower.update();
    }
    @Override
    public void run() {
        super.run();
        robot.updateRobotRun();
        schedule(firstpath.andThen(secondpath));
        robot.driveTrain.follower.update();
    }
}
