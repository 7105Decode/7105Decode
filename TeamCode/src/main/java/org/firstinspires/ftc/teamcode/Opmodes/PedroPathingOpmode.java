package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain.createFollower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Robot.BetterPanels;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

@Autonomous
public class PedroPathingOpmode extends PedroOpMode {
        public PedroPathingOpmode() {
            super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE,BetterPanels.INSTANCE,Shooter.INSTANCE);
}

        private final Pose startPose = new Pose(9.0, 60.0, Math.toRadians(0.0));
        private final Pose finishPose = new Pose(37.0, 50.0, Math.toRadians(180.0));

//        private PathChain move = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, finishPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading())
//                .build();;

        public Command secondRoutine() {

            return new SequentialGroup(
                    new ParallelGroup(
//                            new FollowPath(move),
                            Transfer.INSTANCE.transfer_LeftArtifact()
                    ),
                    new Delay(1),
                    new ParallelGroup(
                            Transfer.INSTANCE.down_LeftArtifact()
                    )
//                    new Delay(1.0),
//                    Lift.INSTANCE.toLow()
            );
        }

        @Override
        public void onInit() {
            createFollower(hardwareMap);
//            DriveTrain.follower.setStartingPose(startPose);
//            buildPaths();
        }

        @Override
        public void onStartButtonPressed() {
            secondRoutine().invoke();
        }
    }

