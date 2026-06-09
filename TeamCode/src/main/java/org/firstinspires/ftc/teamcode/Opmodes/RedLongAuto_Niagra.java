package org.firstinspires.ftc.teamcode.Opmodes;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodUp;
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

import org.firstinspires.ftc.teamcode.BlueLongPaths;
import org.firstinspires.ftc.teamcode.BlueLongPaths_Niagra;
import org.firstinspires.ftc.teamcode.RedLongPaths;
import org.firstinspires.ftc.teamcode.RedLongPaths_Niagra;
import org.firstinspires.ftc.teamcode.Robot.Commands.ArtifactShaker;
import org.firstinspires.ftc.teamcode.Robot.Commands.FishForAprilTagBlueAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.FishForAprilTagRedAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPathTimer;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransferCheckforPark;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Commands.ShooterAutoSlowerSpeed;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnShooterOff;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnTurret_Encoder;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnTurret_Timer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import
@Autonomous(name = "\uD83D\uDFE5LongAuto_Niagra")
public class RedLongAuto_Niagra extends NextFTCOpMode {
    public RedLongAuto_Niagra() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    RedLongPaths_Niagra paths;
    // from the starpose to middle is around -880
    public Command runRobot() {
        return new SequentialGroup(
                //move out and turn the turret to the correct position

                // .865 here is the feedforward for the flywheel you would adjust feedforward to make the wheel go faster,
                // however you would also need to make the -2280 smaller.
                // So for speeding up the wheel a little bit you could try something like
                // .89 for feedforward and -2340
                new ParallelGroup(new RunShooter(.865,-2280),
                        new TurnTurret_Timer(Turret.INSTANCE,1,-920)),
                // below are the transfer servos. So this where we shoot the balls.
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.82),

                //This will go grab off the line you may need to adjust the path slightly.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.GrabLine),
                        new ShooterAutoSlowerSpeed(),
                        new Delay(.6).then(new RunIntakeAuto(true)),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),


                //Return to shoot the artifacts grabbed off the line.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Shoot),
                        new Delay(.8).then(new RunShooter(.87,-2280))),
                new ParallelGroup(new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.82)
                ,new TurnTurret_Timer(Turret.INSTANCE,.4,-945)),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.82),

                //This is going to the corner to grab the arifacts. This path might also need to be adjusted.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.GrabCorner),
                        new TurnShooterOff(),
                        new RunIntakeAuto(true),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                //The paths below are for grabbing the artifact from the corner.
                new FollowPath(DriveTrain.INSTANCE,paths.BackUp),
                new FollowPath(DriveTrain.INSTANCE,paths.PrepShift),
                new ParallelGroup(new FollowPathTimer(DriveTrain.INSTANCE,paths.ShiftLastArtifact,1),new ShooterAutoSlowerSpeed()),

                //Return to shoot.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.ReturnToShoot),
                        new RunShooter(.87,-2280)),
                new ParallelGroup(new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.82)
                        ,new TurnTurret_Timer(Turret.INSTANCE,.3,-920)),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.82),

                //Go to grab more artifacts from corner.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.GrabCorner),
                        new ShooterAutoSlowerSpeed(),
                        new RunIntakeAuto(true),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),
                //Go to shoot.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.ReturnToShoot),
                        new RunShooter(.87,-2280)),
                new ParallelGroup(new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.82)
                        ,new TurnTurret_Timer(Turret.INSTANCE,.3,-920)),
                new MoveTransferCheckforPark(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.38,getRuntime(),100),
                new MoveTransferCheckforPark(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.82,getRuntime(),27.9),
                new MoveTransferCheckforPark(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.38,getRuntime(), 100),

//                This is for park.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Park),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.82)));
    }
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
        Shooter.INSTANCE.hood.setPosition(hoodUp);
        paths = new RedLongPaths_Niagra(DriveTrain.INSTANCE.follower);

    }
    @Override
    public void onWaitForStart() {
        DriveTrain.INSTANCE.drawOnlyCurrent();
        DriveTrain.INSTANCE.updateFollower();
    }
    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.setStartPose(RedLongPaths_Niagra.startpose);
        DriveTrain.INSTANCE.updateFollower();
        runRobot().invoke();
    }
}

