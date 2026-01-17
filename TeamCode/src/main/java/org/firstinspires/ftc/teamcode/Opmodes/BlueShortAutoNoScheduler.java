package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Line;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath_PP;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous
@Configurable
public class BlueShortAutoNoScheduler extends OpMode {
    Robot robot;
    public static boolean firstpath = false, secondpath = false;
    ElapsedTime timer = new ElapsedTime();
//    FollowPath_PP firstpath,secondpath,thirdpath;
    public static Pose startpose, firstpose,secondpose;
    PathChain builder;
    @Override
    public void init() {
        startpose = new Pose(31.4,135.3,180);
        firstpose = new Pose(35.3, 132.9,180);
        secondpose = new Pose(60, 132.7,270);
        robot = new Robot(Robot.OpModeType.AUTO,hardwareMap,startpose,telemetry);
//        builder = robot.driveTrain.follower.pathBuilder()
//                .addPath(new BezierLine(startpose, firstpose))
//                .addPath(new BezierLine(firstpose, secondpose))
//                .build();
//        firs
//        firstpath = new FollowPath_PP(new Path(new BezierLine(startpose,firstpose)),4);
//        secondpath = new FollowPath_PP(new Path(new BezierLine(firstpose,secondpose)),4);
//        thirdpath = new FollowPath_PP();
    }

    @Override
    public void init_loop() {
        robot.driveTrain.follower.activateAllPIDFs();
        robot.driveTrain.follower.update();
        timer.reset();
        robot.initLoopAuto();
    }

    @Override
    public void start() {
        robot.driveTrain.follower.update();
    }

    @Override
    public void loop() {
        if (!robot.driveTrain.follower.isBusy() && timer.seconds() < 1) {
            robot.driveTrain.follower.followPath(new Path(new BezierLine(startpose, firstpose)));
        }
        robot.updateRobotRun();
        robot.driveTrain.follower.update();
    }

    @Override
    public void stop() {
        super.stop();
        robot.driveTrain.follower.setTeleOpDrive(0,0,0);
        robot.driveTrain.follower.update();
    }
}
