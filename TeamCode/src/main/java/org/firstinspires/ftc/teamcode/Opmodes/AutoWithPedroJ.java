package org.firstinspires.ftc.teamcode.Opmodes;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.MoreConvenientTelemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import


@Autonomous(name = "Auto with Pedro (Java)")
public class AutoWithPedroJ extends NextFTCOpMode {
    public AutoWithPedroJ() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, MoreConvenientTelemetry.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    Command command,followFirstPS,followSecondPS,followThirdPS;
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);

        DriveTrain.INSTANCE.follower.setPose(new Pose(0.0, 0.0, 0.0));
        Pose startPose = new Pose(0.0, 0.0, 0.0);
        Pose firstMove   = new Pose(24.0, 0.0, 0.0);
        Pose secondMove   = new Pose(24.0, 0.0, 0.0);
        Pose thirdMove   = new Pose(24.0, 0.0, 0.0);
        Pose fourthMove   = new Pose(24.0, 0.0, 0.0);
        Pose fifthMove   = new Pose(24.0, 0.0, 0.0);
        PathChain pathSequence = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstMove))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstMove.getHeading())
                .build();
        PathChain pathSequence2 = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(firstMove, secondMove))
                .setLinearHeadingInterpolation(firstMove.getHeading(), secondMove.getHeading())
                .build();
        PathChain pathSequence3 = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(firstMove, secondMove))
                .setLinearHeadingInterpolation(firstMove.getHeading(), secondMove.getHeading())
                .build();
        DriveTrain.INSTANCE.follower.followPath(pathSequence);
        DriveTrain.INSTANCE.follower.followPath(pathSequence2);
        DriveTrain.INSTANCE.follower.followPath(pathSequence3);
        command = new SequentialGroup(
//                new a
        );
    }
    @Override
    public void onWaitForStart() {


//        DriveTrain.INSTANCE.follower.followPath(path);
    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {
//        telemetry.addData("Pose", Drfollower.getPose());
        command.invoke();
        DriveTrain.INSTANCE.follower.update();
    }
}

