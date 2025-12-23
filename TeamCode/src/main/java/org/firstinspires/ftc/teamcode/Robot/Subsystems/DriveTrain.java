package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class DriveTrain extends SubsystemBase{
    Follower follower;
    Pose pose;
    public DriveTrain(HardwareMap hardwareMap, Pose pose){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose);
    }
//    public void driveTrainTelem(){
//        BetterDashboard.telemetryPacket.fieldOverlay().setStroke("#4c00b0");
//        Drawing.drawRobot(BetterDashboard.telemetryPacket.fieldOverlay(), pose);
//        BetterDashboard.sendTelem("heading", getHeadingDegrees());
//        BetterDashboard.sendTelem("x",getX());
//        BetterDashboard.sendTelem("y",getY());
//    }
    public double getHeadingDegrees(){
        return Math.toRadians(follower.getHeading());
    }
    public double getX(){
        return follower.getPose().getX();
    }
    public double getY(){
        return follower.getPose().getY();
    }
    public Pose getPose(){
        return follower.getPose();
    }
    public void fieldCentricDrive(Gamepad gamepad){
        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
    }
}
