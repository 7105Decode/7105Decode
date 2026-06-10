package org.firstinspires.ftc.teamcode.Opmodes;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodUp;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midfurtherback;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.RedLongPaths;
import org.firstinspires.ftc.teamcode.Robot.Commands.FishForAprilTagRedAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPathTimer;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnShooterOff;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import
@Disabled
@Autonomous(name = "\uD83D\uDFE5SafeLongAuto")
public class RedSafeLong extends NextFTCOpMode {
    public RedSafeLong() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    RedLongPaths paths;
    // from the starpose to middle is around -880
    public Command runRobot() {
        return new SequentialGroup(

                //move out and turn the turret to the correct position
                new ParallelGroup(new RunShooter(.865,-2280),
                        new FishForAprilTagRedAuto(1,false,.33,-1.6,1.5),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midfurtherback,.4)
                ),
                new Delay(.7),
                // shooting the preloads
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),

                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.GrabCorner),
                        new TurnShooterOff(),
                        new RunIntakeAuto(true),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                new FollowPath(DriveTrain.INSTANCE,paths.BackUp),
                new FollowPath(DriveTrain.INSTANCE,paths.PrepShift),
                new FollowPathTimer(DriveTrain.INSTANCE,paths.ShiftLastArtifact,1),

                new Delay(.7),

                new FollowPathTimer(DriveTrain.INSTANCE,paths.GrabLastArtifact,.6),
                new FollowPath(DriveTrain.INSTANCE,paths.SafePark),
                new RunIntakeAuto(false)
        );

    }
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
        Shooter.INSTANCE.hood.setPosition(hoodUp);
        paths = new RedLongPaths(DriveTrain.INSTANCE.follower);

    }
    @Override
    public void onWaitForStart() {
        DriveTrain.INSTANCE.drawOnlyCurrent();
        DriveTrain.INSTANCE.updateFollower();
    }
    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.setStartPose(RedLongPaths.RedLongStartPose);
        DriveTrain.INSTANCE.updateFollower();
        runRobot().invoke();
    }
}

