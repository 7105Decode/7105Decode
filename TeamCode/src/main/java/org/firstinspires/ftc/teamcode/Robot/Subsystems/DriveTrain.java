package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

public class DriveTrain extends SubsystemBase{
    Follower follower;
    Pose pose;
    public DriveTrain(HardwareMap hardwareMap, Pose pose){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose);
    }
    public void driveTrainTelem(){
        BetterDashboard.telemetryPacket.fieldOverlay().setStroke("#4c00b0");
        Drawing.drawRobot(BetterDashboard.telemetryPacket.fieldOverlay(), pose);
        BetterDashboard.sendTelem("heading", getHeadingDegrees());
        BetterDashboard.sendTelem("x",getX());
        BetterDashboard.sendTelem("y",getY());
    }
    public double getHeadingDegrees(){
        return Math.toDegrees(pose.heading.toDouble());
    }
    public double getX(){
        return pose.position.x;
    }
    public double getY(){
        return pose.position.y;
    }
    public Pose2d getPose(){
        return pinpointDrive.pose;
    }
    public void fieldCentricDrive(Gamepad gamepad){
        pinpointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x
                ),
                -gamepad.right_stick_x
        ));
    }
}
