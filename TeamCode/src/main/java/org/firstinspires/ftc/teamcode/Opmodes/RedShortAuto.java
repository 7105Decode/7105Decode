package org.firstinspires.ftc.teamcode.Opmodes;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodUp;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.apache.commons.math3.geometry.Point;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.MoveTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.ReadObelisk;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunShooter;
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
    Command readObelisk(){
        return new ReadObelisk(Turret.INSTANCE,telemetry);
    }
    public Command runRobot() {
        return new SequentialGroup(
                new ParallelGroup(new RunShooter(Shooter.INSTANCE,.03,-2280),
                        new FollowPath(Paths.RedShortPath1)),
                new MoveTransfer(true,false,false,.7),
                new Delay(.95),
                new MoveTransfer(true,false,false,Transfer.rightdownpos),
                new Delay(.7),
                new MoveTransfer(false,true,false,Transfer.lefttransferpos),
                new Delay(.95),
                new MoveTransfer(false,true,false,Transfer.leftdownpos),
                new Delay(.7),
                new MoveTransfer(false,false,true, midreadytransferpos),
                new Delay(.95),
                new MoveTransfer(false,false,true,Transfer.middownpos),
                new Delay(.7)
        );
    }
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
        Shooter.INSTANCE.hood.setPosition(hoodUp);

    }
    @Override
    public void onWaitForStart() {
        DriveTrain.INSTANCE.drawOnlyCurrent();
        DriveTrain.INSTANCE.updateFollower();
        readObelisk().invoke();
    }
    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.resetEncoder();
        DriveTrain.INSTANCE.follower.setPose(Paths.RedShortStartPose);
        runRobot().invoke();
    }
}

