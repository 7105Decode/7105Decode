package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Line;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath_PP;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous
@Disabled
@Configurable
public class BlueShortAuto extends CommandOpMode {
    Robot robot;
    FollowPath_PP firstpath,secondpath;
    Pose startpose, firstpose,secondpose;
   Command addthemall;
    @Override
    public void initialize() {
        startpose = new Pose(31.4,135.3,180);
        firstpose = new Pose(35.3, 132.9,180);
        secondpose = new Pose(60, 132.7,270);
        robot = new Robot(Robot.OpModeType.AUTO,hardwareMap,startpose,telemetry);
        firstpath = new FollowPath_PP(robot.driveTrain.follower,new Path(new BezierLine(startpose,firstpose)),4);
        secondpath = new FollowPath_PP(robot.driveTrain.follower,new Path(new BezierLine(firstpose,secondpose)),4);
//        thirdpath = new FollowPath_PP();
        addthemall = firstpath.andThen(secondpath);

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
        schedule(addthemall);
        robot.driveTrain.follower.update();
    }
    @Override
    public void run() {
        super.run();
        robot.updateRobotRun();
        robot.driveTrain.follower.update();
    }
}
