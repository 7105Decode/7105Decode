package org.firstinspires.ftc.teamcode.Opmodes;


import static org.firstinspires.ftc.teamcode.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.MoreConvenientTelemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

import dev.nextftc.extensions.pedro.PedroComponent;
// optional static import


@Autonomous(name = "Auto with Pedro (Java)")
public class AutoWithPedroJ extends NextFTCOpMode {
    public AutoWithPedroJ() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, MoreConvenientTelemetry.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
    }
    Command command;
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);

        DriveTrain.INSTANCE.follower.setPose(new Pose(0.0, 0.0, 0.0));
        Pose start = new Pose(0.0, 0.0, 0.0);
        Pose end   = new Pose(24.0, 0.0, 0.0);
        PathChain path = DriveTrain.INSTANCE.follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        command = new FollowPath(path);
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

