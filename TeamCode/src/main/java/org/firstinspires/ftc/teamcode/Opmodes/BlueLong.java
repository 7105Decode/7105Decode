package org.firstinspires.ftc.teamcode.Opmodes;


import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.lefttransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.midtransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.righttransferservopos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodUp;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.leftreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransferDown;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTurret;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooterNoController;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnShooterOff;
import org.firstinspires.ftc.teamcode.Robot.MoreConvenientTelemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import


@Autonomous(name = "Auto with Pedro (Java)")
public class BlueLong extends NextFTCOpMode {
    public BlueLong() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, MoreConvenientTelemetry.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    Command command,followFirstPS,followSecondPS,followThirdPS;
    Pose startPose, firstMove,secondMove,thirdMove,fourthMove,fifthMove;
    PathChain pathSequence,pathSequence2,pathSequence3,pathSequence4;
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
        Shooter.INSTANCE.hood.setPosition(hoodUp);
        Turret.INSTANCE.resetEncoder();

         startPose = new Pose(59, 9, Math.toRadians(180));
         firstMove   = new Pose(42, 35.7, Math.toRadians(180));
         secondMove   = new Pose(16.8, 35.5, Math.toRadians(180));
         thirdMove   = new Pose(59, 9, Math.toRadians(180));
         fourthMove   = new Pose(24.0, 0.0, 0.0);
         fifthMove   = new Pose(24.0, 0.0, 0.0);
         pathSequence = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstMove))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstMove.getHeading())
                .build();
         pathSequence2 = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(firstMove, secondMove))
                .setLinearHeadingInterpolation(firstMove.getHeading(), secondMove.getHeading())
                .build();
         pathSequence3 = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(secondMove, thirdMove))
                .setLinearHeadingInterpolation(secondMove.getHeading(), thirdMove.getHeading())
                .build();
        pathSequence4 = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(secondMove, thirdMove))
                .setLinearHeadingInterpolation(secondMove.getHeading(), thirdMove.getHeading())
                .build();
//        DriveTrain.INSTANCE.follower.followPath(pathSequence);
//        DriveTrain.INSTANCE.follower.followPath(pathSequence2);
//        DriveTrain.INSTANCE.follower.followPath(pathSequence3);
    }
    @Override
    public void onWaitForStart() {
        command = new ParallelGroup(
//                        new MoveTurret(955).then(Turret.INSTANCE.runPID()),
                new RunShooterNoController().then(
                new org.firstinspires.ftc.teamcode.Robot.Commands.Delay(.5)).then(
                new MoveTransfer(true,false,false,righttransferpos)).then(
                new org.firstinspires.ftc.teamcode.Robot.Commands.Delay(1.2)).then(
                new MoveTransferDown(true,false,false,righttransferservopos)).then(
                new Delay(1.2)));
//                new MoveTransfer(false,true,false,leftreadytransferpos),
//                new Delay(1.2),
//                new MoveTransfer(false,true,false,lefttransferservopos),
//                new Delay(1.2),
//                new MoveTransfer(false,false,true,midreadytransferpos),
//                new Delay(1.2),
//                new MoveTransfer(false,false,true,midtransferservopos),
//                new Delay(1.2),
//                new ParallelGroup(new FollowPath(pathSequence), new TurnShooterOff()),
//                new ParallelGroup(new FollowPath(pathSequence2),new RunIntakeAuto(true)),
//
//                new ParallelGroup(new FollowPath(pathSequence3),new RunIntakeAuto(false)
//                        ,new MoveTurret(955).then(Turret.INSTANCE.runPID())
//                        ,new RunShooterNoController(Shooter.INSTANCE,.87)),
//                new MoveTransfer(true,false,false,righttransferpos),
//                new MoveTransfer(false,true,false,leftreadytransferpos),
//                new MoveTransfer(false,false,true,midreadytransferpos),
//                new ParallelGroup(new MoveTurret(0).then(Turret.INSTANCE.runPID()), new TurnShooterOff())
//
//        );
    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.follower.setPose(startPose);
    }

    @Override
    public void onUpdate() {
        command.invoke();
        DriveTrain.INSTANCE.updateFollower();
    }
}

