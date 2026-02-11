package org.firstinspires.ftc.teamcode.Opmodes;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.hoodDown;
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
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeAuto;
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
    Paths paths;
    Command readObelisk(){
        return new ReadObelisk(Turret.INSTANCE,telemetry);
    }
    public Command runRobot() {
        return new SequentialGroup(
                new ParallelGroup(new RunShooter(Shooter.INSTANCE,0.012,-1560),
                        new FollowPath(paths.RedShortPath1)),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,.7,.95),
                new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,2),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,2),
                new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95),
                new ParallelGroup(new FollowPath(paths.RedShortPath2),
                new RunShooter(Shooter.INSTANCE,.008,-1000),
                new RunIntakeAuto(true),
                new MoveTransfer(Transfer.INSTANCE,false,false,true,Transfer.middownpos,2)),
                new FollowPath(paths.RedShortPath3)
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

