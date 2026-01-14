
package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.Opmodes.Tele.hoodup;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.kd;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.ki;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.kp;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.lefttransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.midtransferservopos;
import static org.firstinspires.ftc.teamcode.Opmodes.Tele.righttransferservopos;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous
public class MaybeBetterAutoRedSide extends LinearOpMode {
    Follower follower;
    DcMotor rightshooter,leftshooter,topturret;
    Servo righttransfer, midtransfer,lefttransfer, hood;
    BasicPID pid;
    ElapsedTime timer = new ElapsedTime();
    public static PIDCoefficients pidCoefficients;
    @Override
    public void runOpMode() throws InterruptedException {
        rightshooter = hardwareMap.get(DcMotor.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotor.class,"leftshooter");
        hood = hardwareMap.get(Servo.class,"hood");
        topturret = hardwareMap.get(DcMotor.class,"topturret");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72));
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
        follower.startTeleopDrive(true);
        follower.update();
        while (opModeIsActive()){
            if (timer.seconds()<3){
                topturret.setPower(pid.calculate(-925,topturret.getCurrentPosition()));
                rightshooter.setPower(.88);
                leftshooter.setPower(.88);
            } else if (timer.seconds() < 6) {
                lefttransfer.setPosition(.7);}
            else if (timer.seconds() < 7) {
                lefttransfer.setPosition(lefttransferservopos);
            } else if (timer.seconds() < 10) {
                midtransfer.setPosition(.7);
            } else if (timer.seconds() < 11) {
                midtransfer.setPosition(midtransferservopos);
            } else if (timer.seconds() < 14) {
                righttransfer.setPosition(.7);
            } else if (timer.seconds() < 17) {
                righttransfer.setPosition(righttransferservopos);
                topturret.setPower(pid.calculate(0, topturret.getCurrentPosition()));
                rightshooter.setPower(0);
                leftshooter.setPower(0);

            }else if  (timer.seconds() < 20){
                    follower.setTeleOpDrive(.3, 0, 0);
                    follower.update();
                }
                else {
                follower.setTeleOpDrive(0, 0, 0);
                follower.update();
            }

        }
    }
}
