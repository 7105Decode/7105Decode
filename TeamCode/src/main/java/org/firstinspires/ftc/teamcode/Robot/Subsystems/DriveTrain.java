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
import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() { }
    public Follower follower;
    @IgnoreConfigurable
    static PoseHistory poseHistory;
    public DriveTrain(HardwareMap hardwareMap, Pose pose){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose);
        follower.startTeleopDrive(true);
        follower.update();
        poseHistory = follower.getPoseHistory();
    }
    public void initloop(){
        drawOnlyCurrent();
    }

    @Override
    public void periodic() {
        driveTrainTelem();
    }

    public void driveTrainTelem(){
        draw();
        BetterDashboard.addtelem("heading", getHeadingDegrees());
        BetterDashboard.addtelem("x",getX());
        BetterDashboard.addtelem("y",getY());
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
