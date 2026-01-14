package org.firstinspires.ftc.teamcode.Opmodes;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous
public class Auto extends LinearOpMode {
    Follower follower;
    DcMotor rightshooter,leftshooter;
    Servo righttransfer, midtransfer,lefttransfer, hood;
    BasicPID pid;
    public static PIDCoefficients pidCoefficients;
    @Override
    public void runOpMode() throws InterruptedException {
        rightshooter = hardwareMap.get(DcMotor.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotor.class,"leftshooter");
        hood = hardwareMap.get(Servo.class,"hood");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hood.setPosition(hoodup);
        waitForStart();
//        rightshooter.setPower(1);
//        leftshooter.setPower(1);
//        sleep(2500);
//        lefttransfer.setPosition(.7);
//        sleep(3000);
//        lefttransfer.setPosition(lefttransferservopos);
//        sleep(1000);
//        midtransfer.setPosition(.7);
//        sleep(3000);
//        midtransfer.setPosition(midtransferservopos);
//        sleep(1000);
//        righttransfer.setPosition(.7);
//        sleep(3000);
//        righttransfer.setPosition(righttransferservopos);
//        rightshooter.setPower(0);
//        leftshooter.setPower(0);

        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(.3,0,0);
        follower.update();
        sleep(3000);
        follower.setTeleOpDrive(0,0,0);
        follower.update();
        while (opModeIsActive()) {

        }
    }
}
