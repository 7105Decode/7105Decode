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
import org.firstinspires.ftc.teamcode.Robot.Commands.ArtifactShaker;
import org.firstinspires.ftc.teamcode.Robot.Commands.FishForAprilTagBlueAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.FishForAprilTagRedAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPathTimer;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
import org.firstinspires.ftc.teamcode.Robot.Commands.ShooterAutoSlowerSpeed;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnShooterOff;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurnTurret_Encoder;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
// optional static import
@Autonomous(name = "\uD83D\uDFE6LongAuto_Niagra")
public class BlueLongAuto_Niagra extends NextFTCOpMode {
    public BlueLongAuto_Niagra() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    BlueLongPaths_Niagra paths;
    // from the starpose to middle is around -880
    public Command runRobot() {
        return new SequentialGroup(
                //move out and turn the turret to the correct position

                // .865 here is the feedforward for the flywheel you would adjust feedforward to make the wheel go faster,
                // however you would also need to make the -2280 smaller.
                // So for speeding up the wheel a little bit you could try something like
                // .89 for feedforward and -2340
                new ParallelGroup(new RunShooter(865,-3200),
                        new TurnTurret_Encoder(Turret.INSTANCE,900,.7)),
                new Delay(.7),
                // below are the transfer servos. So this where we shoot the balls.
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.82),

                //This will go grab off the like you may need to adjust the path slightly.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path1),
                        new TurnShooterOff(),
                        new Delay(.2).then(new ShooterAutoSlowerSpeed()),
                        new Delay(.6).then(new RunIntakeAuto(true)),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),


                //Return to shoot the artifacts grabbed off the line.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path2),
                        new TurnTurret_Encoder(Turret.INSTANCE,900,.4),
                        new RunShooter(.87,-2280)),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.82),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.81),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.38),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.81),

                //This is going to the corner to grab the arifacts. This path might also need to be adjusted.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path3),
                        new TurnShooterOff(),
                        new RunIntakeAuto(true),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),

                //The paths below are for grabbing the artifact from the corner.
                new FollowPath(DriveTrain.INSTANCE,paths.Path4),
                new ParallelGroup(new FollowPathTimer(DriveTrain.INSTANCE,paths.Path5,1),new RunShooter(.87,-2280)),
//                new FollowPathTimer(DriveTrain.INSTANCE,paths.Path6,1),

                //Return to shoot.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path7),
                        new TurnTurret_Encoder(Turret.INSTANCE,900,.3)),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.81),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.37),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.81),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.37),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.81),

                //Go to grab more artifacts from corner.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path8),
                        new ShooterAutoSlowerSpeed(),
                        new Delay(.3).then(new RunShooter(.87,-2280)),
                        new RunIntakeAuto(true),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,.2)),
                //Go to shoot.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path9),
                        new TurnTurret_Encoder(Turret.INSTANCE,900,.3),
                        new RunShooter(.87,-2280)),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.81),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.37),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.81),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.37),

//                This is for park.
                new ParallelGroup(new FollowPath(DriveTrain.INSTANCE,paths.Path10),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.82)));
    }
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
        Shooter.INSTANCE.hood.setPosition(hoodUp);
        paths = new BlueLongPaths_Niagra(DriveTrain.INSTANCE.follower);

    }
    @Override
    public void onWaitForStart() {
        DriveTrain.INSTANCE.drawOnlyCurrent();
        DriveTrain.INSTANCE.updateFollower();
    }
    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.setStartPose(BlueLongPaths_Niagra.startpose);
        DriveTrain.INSTANCE.updateFollower();
        runRobot().invoke();
    }
}

