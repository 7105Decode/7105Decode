package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.sun.tools.doclint.Entity.pi;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

public class DriveTrain extends SubsystemBase{
    PinpointDrive pinpointDrive;
    Pose2d pose;
    public DriveTrain(HardwareMap hardwareMap, Pose2d pose2d){
        pinpointDrive = new PinpointDrive(hardwareMap, pose2d);
        pose = pinpointDrive.pose;
    }
    public void driveTrainTelem(BetterDashboard betterDashboard){
        betterDashboard.telemetryPacket.fieldOverlay().setStroke("#4c00b0");
        Drawing.drawRobot(betterDashboard.telemetryPacket.fieldOverlay(), pose);
        betterDashboard.sendTelem("heading", getHeadingDegrees());
        betterDashboard.sendTelem("x",getX());
        betterDashboard.sendTelem("y",getY());
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
