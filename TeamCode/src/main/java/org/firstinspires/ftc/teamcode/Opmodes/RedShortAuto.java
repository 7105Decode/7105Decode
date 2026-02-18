package org.firstinspires.ftc.teamcode.Opmodes;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodDown;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.middownpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midfurtherback;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTurretWithEncoderThreshold;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.ReadObelisk;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Commands.TrackAprilTag_BangBangAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnShooterOff;
import org.firstinspires.ftc.teamcode.Robot.Paths;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import
@Autonomous(name = "ShortAuto")
public class RedShortAuto extends NextFTCOpMode {
    public RedShortAuto() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    Paths paths;
    Command readObelisk(){
        return new ReadObelisk(0,telemetry);
    }
    // from the starpose to middle is around -880
    public Command runRobot() {
        return new SequentialGroup(

                //move out and turn the turret to the correct position
                new ParallelGroup(new RunShooter(Shooter.INSTANCE,0.012,-1560),
                        new FollowPath(DriveTrain.INSTANCE,paths.RedShortPreloads),
                        new MoveTurretWithEncoderThreshold(Turret.INSTANCE,false,-790,40),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midfurtherback,.4)
                ),

                // shooting the preloads
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortCollectPPG),
                new TurnShooterOff(),
                new RunIntakeAuto(true),
                new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortScorePPG),
                        new RunShooter(Shooter.INSTANCE,0.012,-1560),
                        new SequentialGroup(new MoveTurretWithEncoderThreshold(Turret.INSTANCE,false,-1570,40),
                                new TrackAprilTag_BangBangAuto(1)),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midfurtherback,.4)
                ),

                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95),

        new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortCollectPGP),
                new TurnShooterOff(),
                new RunIntakeAuto(true),
                new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.RedShortScorePGP),
                        new RunShooter(Shooter.INSTANCE,0.012,-1560),
                                new TrackAprilTag_BangBangAuto(1),






                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midfurtherback,.4)
                ),

        new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95)
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
        readObelisk().invoke();
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.setStartPose(Paths.RedShortStartPose);
        DriveTrain.INSTANCE.updateFollower();
        runRobot().invoke();
    }
}

