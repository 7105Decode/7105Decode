//package org.firstinspires.ftc.teamcode.Opmodes;
//
//
//import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodUp;
//import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
//
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
//
//import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
//import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
//import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooterNoController;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
//// optional static import
//
//
//@Autonomous(name = "Auto with Pedro (Java)")
//public class BlueLong extends NextFTCOpMode {
//    public BlueLong() {
//        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
//    }
//    Pose startPose, firstMove,secondMove,thirdMove,fourthMove,fifthMove;
//    PathChain pathSequence,pathSequence2,pathSequence3,pathSequence4;
//    public Command runRobot() {
//        return new SequentialGroup(
////                new RunShooter(),
//                new MoveTransfer(true,false,false,.7),
//                new Delay(.95),
//                new MoveTransfer(true,false,false,Transfer.rightdownpos),
//                new Delay(.7),
//                new MoveTransfer(false,true,false,Transfer.lefttransferpos),
//                new Delay(.95),
//                new MoveTransfer(false,true,false,Transfer.leftdownpos),
//                new Delay(.7),
//                new MoveTransfer(false,false,true, midreadytransferpos),
//                new Delay(.95),
//                new MoveTransfer(false,false,true,Transfer.middownpos),
//                new Delay(.7)
//        );
//    }
//    @Override
//    public void onInit() {
//        DriveTrain.INSTANCE.createFollower(hardwareMap);
//        Shooter.INSTANCE.hood.setPosition(hoodUp);
//        Turret.INSTANCE.resetEncoder();
//
//         startPose = new Pose(59, 9, Math.toRadians(180));
//         firstMove   = new Pose(42, 35.7, Math.toRadians(180));
//         secondMove   = new Pose(16.8, 35.5, Math.toRadians(180));
//         thirdMove   = new Pose(59, 9, Math.toRadians(180));
//         fourthMove   = new Pose(24.0, 0.0, 0.0);
//         fifthMove   = new Pose(24.0, 0.0, 0.0);
//    }
//    @Override
//    public void onWaitForStart() {
//        DriveTrain.INSTANCE.drawOnlyCurrent();
//        DriveTrain.INSTANCE.updateFollower();
//        pathSequence = DriveTrain.INSTANCE.follower.pathBuilder()
//                .addPath(new BezierLine(startPose, firstMove))
//                .setLinearHeadingInterpolation(startPose.getHeading(), firstMove.getHeading())
//                .build();
//        pathSequence2 = DriveTrain.INSTANCE.follower.pathBuilder()
//                .addPath(new BezierLine(firstMove, secondMove))
//                .setLinearHeadingInterpolation(firstMove.getHeading(), secondMove.getHeading())
//                .build();
//        pathSequence3 = DriveTrain.INSTANCE.follower.pathBuilder()
//                .addPath(new BezierLine(secondMove, thirdMove))
//                .setLinearHeadingInterpolation(secondMove.getHeading(), thirdMove.getHeading())
//                .build();
//        pathSequence4 = DriveTrain.INSTANCE.follower.pathBuilder()
//                .addPath(new BezierLine(secondMove, thirdMove))
//                .setLinearHeadingInterpolation(secondMove.getHeading(), thirdMove.getHeading())
//                .build();
//    }
//    @Override
//    public void onStartButtonPressed() {
//        Turret.INSTANCE.resetEncoder();
//        DriveTrain.INSTANCE.follower.setPose(startPose);
//        runRobot().invoke();
//    }
//}
//
