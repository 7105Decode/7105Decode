package org.firstinspires.ftc.teamcode.Opmodes;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.feedforwardshort;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodDown;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.middownpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midfurtherback;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.ArtifactShaker;
import org.firstinspires.ftc.teamcode.Robot.Commands.FishForAprilTagRedAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnShooterOff;
import org.firstinspires.ftc.teamcode.Robot.Paths;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import
@Autonomous(name = "\uD83D\uDFE5ShortAuto")
public class RedShortAuto extends NextFTCOpMode {
    public RedShortAuto() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    Paths paths;
    // from the starpose to middle is around -880
    public Command runRobot() {
        return new SequentialGroup(

                //move out and turn the turret to the correct position
                new ParallelGroup(new RunShooter(feedforwardshort,-1650),
                        new FollowPath(DriveTrain.INSTANCE,paths.RedShortPreloads),
                        new FishForAprilTagRedAuto(1,false,.33,-7,3),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midfurtherback,.4)
                ),

                // shooting the preloads
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortCollectPPG),
                new TurnShooterOff(),
                new RunIntakeAuto(true),
                new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortScorePPG),
                        new RunShooter(feedforwardshort,-1600),
                        new FishForAprilTagRedAuto(1,false,.3, -4.5,2),
                        new Delay(1.5).then( new ArtifactShaker()),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.4)
                ),

                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),

        new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortCollectPGP),
                new TurnShooterOff(),
                new RunIntakeAuto(true),
                new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortScorePGP),
                        new RunShooter(.67,-1800),
                        new Delay(1.5).then( new ArtifactShaker()),
                        new Delay(1.9).then(new FishForAprilTagRedAuto(1,true,.3, -7.5,2)),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.4)
                ),
        new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortCollectGPP),
                        new TurnShooterOff(),
                        new RunIntakeAuto(true),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2))
        );
    }
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
        Shooter.INSTANCE.hood.setPosition(hoodDown);
        paths = new Paths(DriveTrain.INSTANCE.follower);

    }
    @Override
    public void onWaitForStart() {
        DriveTrain.INSTANCE.drawOnlyCurrent();
        DriveTrain.INSTANCE.updateFollower();


    }
    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.setStartPose(Paths.RedShortStartPose);
        DriveTrain.INSTANCE.updateFollower();
        runRobot().invoke();
    }
}

