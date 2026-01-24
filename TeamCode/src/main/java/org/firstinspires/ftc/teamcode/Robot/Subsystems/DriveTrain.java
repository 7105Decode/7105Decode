package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rowanmcalpin.nextftc.core.Subsystem;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DrawingCopy;
import org.firstinspires.ftc.teamcode.Robot.BetterPanels;

public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() { }
    public static Follower follower;
    @IgnoreConfigurable
    static PoseHistory poseHistory;
//    new Path

    public static void createFollower(HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void initialize() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(pose);
    }

    public void setStartPose(Pose pose){
        follower.setStartingPose(pose);
    }

    @Override
    public void periodic() {
        driveTrainTelem();
    }

    public void driveTrainTelem(){
        draw();
        BetterPanels.addtelem("heading", getHeadingDegrees());
        BetterPanels.addtelem("x",getX());
        BetterPanels.addtelem("y",getY());
    }
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
    public void drawOnlyCurrent() {
        try {
            DrawingCopy.drawRobot(follower.getPose());
            DrawingCopy.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
    public void draw() {
        DrawingCopy.drawDebug(follower);
    }
}
