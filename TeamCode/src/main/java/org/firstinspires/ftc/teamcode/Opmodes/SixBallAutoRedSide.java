package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.Opmodes.Tele.hoodup;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.kd;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.ki;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.kp;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.lefttransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.midtransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.righttransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.shooterkp;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleBlueBetter.leftvel;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.targetvel;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;

//750
@Autonomous
@Configurable
public class SixBallAutoRedSide extends LinearOpMode {
    Follower follower;
    DcMotorEx rightshooter,leftshooter,topturret,intake;
    Servo righttransfer, midtransfer,lefttransfer, hood;
    BasicPID pid, shooterpid;
    ElapsedTime timer = new ElapsedTime();
    public static PIDCoefficients pidCoefficients, shooterCoef;

    Pose startpose, firstMove,secondMove,thirdMove,fourthMove;
    public static double firsty = 98, firstx = 89,secondx = 109, secondy = 100,thirdx = 109, thirdy = 100,fourthx = 123.8, fourthy = 73.7;

    @Override
    public void runOpMode() throws InterruptedException {
        rightshooter = hardwareMap.get(DcMotorEx.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotorEx.class,"leftshooter");
        hood = hardwareMap.get(Servo.class,"hood");
        topturret = hardwareMap.get(DcMotorEx.class,"topturret");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");
        intake = hardwareMap.get(DcMotorEx.class,"frontintake");

        follower = Constants.createFollower(hardwareMap);
        startpose = new Pose(72,72);
        firstMove = new Pose(firstx,firsty);
        secondMove = new Pose(secondx,secondy);
        thirdMove = new Pose(thirdx,thirdy);
        fourthMove = new Pose(fourthx,fourthy);
        follower.setStartingPose(startpose);
        follower.update();
        righttransfer.setDirection(Servo.Direction.REVERSE);
        rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topturret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood.setPosition(hoodup);
        PathChain pathSequence = follower.pathBuilder()
                .addPath(new BezierLine(startpose, firstMove))
                .setConstantHeadingInterpolation(startpose.getHeading())
                .build();
        PathChain pathSequence2 = follower.pathBuilder()
                .addPath(new BezierLine(firstMove, secondMove))
                .setConstantHeadingInterpolation(startpose.getHeading())
                .build();
        PathChain pathSequence3 = follower.pathBuilder()
                .addPath(new BezierLine(secondMove, startpose))
                .setConstantHeadingInterpolation(startpose.getHeading())
                .build();
        waitForStart();
        follower.setStartingPose(startpose);
        follower.update();
        timer.reset();
        pidCoefficients = new PIDCoefficients(kp,ki,kd);
        pid = new BasicPID(pidCoefficients);
        shooterkp = .024;
        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
        shooterpid = new BasicPID(shooterCoef);
        targetvel = -2280;
        follower.followPath(pathSequence,true);
        while (opModeIsActive()){
            leftvel = leftshooter.getVelocity();
            if (timer.seconds()<1.8){
                topturret.setPower(pid.calculate(-935,topturret.getCurrentPosition()));
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
            } else if (timer.seconds() < 2.9) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(.7);}
            else if (timer.seconds() < 3.3) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(lefttransferservopos);
            } else if (timer.seconds() < 3.7) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(.7);
            } else if (timer.seconds() < 4.8) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(midtransferservopos);
            } else if (timer.seconds() < 5.2) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                righttransfer.setPosition(.7);
            } else if (timer.seconds() < 6.3) {
                righttransfer.setPosition(righttransferservopos);
                rightshooter.setPower(0.25);
                leftshooter.setPower(0.25);
            } else if  (timer.seconds() < 9){
                follower.update();
            } else if (timer.seconds() < 9.05){
                intake.setPower(1);
                follower.breakFollowing();
            }else if (timer.seconds() < 9.1){
                intake.setPower(1);
                follower.followPath(pathSequence2,true);
            }else if  (timer.seconds() < 12){
//                topturret.setPower(pid.calculate(0,topturret.getCurrentPosition()));
                intake.setPower(1);
                follower.update();
            }else if (timer.seconds() < 12.05){
                intake.setPower(1);
                follower.breakFollowing();
            }else if (timer.seconds() < 12.1){
                intake.setPower(1);
                follower.followPath(pathSequence3,true);
            } else if (timer.seconds() < 15){
                intake.setPower(1);
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                follower.update();
            }

            else if (timer.seconds() < 16.1) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(.7);}
            else if (timer.seconds() < 16.6) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(lefttransferservopos);
            } else if (timer.seconds() < 17.7) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(.7);
            } else if (timer.seconds() < 18.2) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(midtransferservopos);
            } else if (timer.seconds() < 19.3) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                righttransfer.setPosition(.7);
            } else if (timer.seconds() < 19.8) {
                rightshooter.setPower(0);
                leftshooter.setPower(0);
                topturret.setPower(pid.calculate(0,topturret.getCurrentPosition()));
                righttransfer.setPosition(righttransferservopos);
            } else if (timer.seconds() < 19.9){
                follower.breakFollowing();
            }else if (timer.seconds() < 20){
                follower.followPath(pathSequence,true);
            }else if  (timer.seconds() < 25){
//                topturret.setPower(pid.calculate(0,topturret.getCurrentPosition()));
                follower.update();
            }else {
                topturret.setPower(pid.calculate(0,topturret.getCurrentPosition()));
                follower.update();
            }

        }
    }
}
