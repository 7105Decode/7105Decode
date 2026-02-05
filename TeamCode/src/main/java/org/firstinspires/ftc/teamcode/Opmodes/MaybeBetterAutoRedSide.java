
package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.Opmodes.Tele.hoodup;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.kd;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.ki;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.kp;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.lefttransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.midtransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.righttransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.shooterkp;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleBlueMaybeBetter.leftvel;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.targetvel;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous
public class MaybeBetterAutoRedSide extends LinearOpMode {
    Follower follower;
    DcMotorEx rightshooter,leftshooter,topturret, frontintake;
    Servo righttransfer, midtransfer,lefttransfer, hood;
    BasicPID pid, shooterpid;
    ElapsedTime timer = new ElapsedTime();
    Pose startPose;
    boolean endPathStarted = false;
    public static PIDCoefficients pidCoefficients, shooterCoef;
    @Override
    public void runOpMode() throws InterruptedException {
        rightshooter = hardwareMap.get(DcMotorEx.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotorEx.class,"leftshooter");
        hood = hardwareMap.get(Servo.class,"hood");
        topturret = hardwareMap.get(DcMotorEx.class,"topturret");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");
        frontintake = hardwareMap.get(DcMotorEx.class, "frontintake");

        follower = Constants.createFollower(hardwareMap);
        startPose = new Pose(77,5, Math.toRadians(0));
        follower.setStartingPose(startPose);
        follower.update();
        righttransfer.setDirection(Servo.Direction.REVERSE);
        rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topturret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood.setPosition(hoodup);
        waitForStart();
        timer.reset();
        pidCoefficients = new PIDCoefficients(kp,ki,kd);
        pid = new BasicPID(pidCoefficients);
        shooterkp = .024;
        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
        shooterpid = new BasicPID(shooterCoef);
        follower.startTeleopDrive(true);
        follower.update();
        targetvel = -2280;
        while (opModeIsActive()){
            leftvel = leftshooter.getVelocity();
            if (timer.seconds()<1){
                topturret.setPower(pid.calculate(-935,topturret.getCurrentPosition()));
            } else if (timer.seconds()<1.5) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
            } else if (timer.seconds() < 3) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(.7);}
            else if (timer.seconds() < 4.5) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(lefttransferservopos);
            } else if (timer.seconds() < 6) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(.7);
            } else if (timer.seconds() < 7.5) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(midtransferservopos);
            } else if (timer.seconds() < 9) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                righttransfer.setPosition(.7);
            } else if (timer.seconds() < 10.5) {
                rightshooter.setPower(0);
                leftshooter.setPower(0);
                righttransfer.setPosition(righttransferservopos);
                topturret.setPower(pid.calculate(0,topturret.getCurrentPosition()));
                rightshooter.setPower(0);
                leftshooter.setPower(0);

            } else if  (timer.seconds() < 16.7){
                follower.setTeleOpDrive(0, .3, 0);
//                follower.followPath(new Path(new BezierLine(new Pose(77,5, Math.toRadians(0)),new Pose(77,12, Math.toRadians(0)))));
                follower.update();
            } else if (timer.seconds() < 19.0) {
                // strafe right while spinning intake
                follower.setTeleOpDrive(0, .3, 0);
                frontintake.setPower(1);
                follower.update();
            } else if (timer.seconds() < 20.3) {
                // stop intake and spin shooters up
                frontintake.setPower(0);
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                follower.setTeleOpDrive(0, 0, 0);
                follower.update();
            } else if (timer.seconds() < 22.3) {
                // keep shooters spinning while raising transfer servos
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(.7);
                midtransfer.setPosition(.7);
                righttransfer.setPosition(.7);
                follower.setTeleOpDrive(0, 0, 0);
                follower.update();
            } else {
                // stop shooters and start following a path home if not started
                rightshooter.setPower(0);
                leftshooter.setPower(0);
                if (!endPathStarted) {
                    endPathStarted = true;
                    Path goHome = new Path(new BezierLine(follower.getPose(), startPose));
                    follower.followPath(goHome);
                }
                if (!follower.atParametricEnd()) {
                    follower.update();
                } else {
                    // reached home pose; move forward briefly then stop
                    double timeSinceHome = timer.seconds() - 20.3;
                    if (timeSinceHome < 1.0) {
                        follower.setTeleOpDrive(0.3, 0, 0);
                    } else {
                        // Final stop: zero all drive and shooter/intake power, update follower, then exit
                        follower.setTeleOpDrive(0, 0, 0);
                        rightshooter.setPower(0);
                        leftshooter.setPower(0);
                        frontintake.setPower(0);
                        follower.update();
                        return;
                    }
                    follower.update();
                }
            }

        }
    }
}
