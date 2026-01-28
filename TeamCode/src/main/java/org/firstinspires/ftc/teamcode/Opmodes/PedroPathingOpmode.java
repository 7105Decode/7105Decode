package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain.createFollower;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.startlimelight;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Robot.BetterPanels;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

@Autonomous
public class PedroPathingOpmode extends PedroOpMode {
        public PedroPathingOpmode() {
            super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE,BetterPanels.INSTANCE,Shooter.INSTANCE, Intake.INSTANCE);
        }
        private final Pose startPose = new Pose(56.0, 8, Math.toRadians(90.0));
        private final Pose finishPose = new Pose(23, 23, Math.toRadians(180.0));
        Path path = new Path(new BezierLine(startPose,finishPose));
        FollowPath followPath = new FollowPath(DriveTrain.INSTANCE,path);
        public Command secondRoutine() {
            return new SequentialGroup(
                    followPath);
        }
        @Override
        public void onInit() {
            createFollower(hardwareMap);
            startlimelight();
        }

        @Override
        public void onStartButtonPressed() {
            secondRoutine().invoke();
        }
    }

