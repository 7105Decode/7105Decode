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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class AutoRedLimelightPreSense extends LinearOpMode {
    Follower follower;
    DcMotorEx rightshooter,leftshooter,topturret;
    Servo righttransfer, midtransfer,lefttransfer, hood;
    BasicPID pid, shooterpid;
    ElapsedTime timer = new ElapsedTime();
    public static boolean greenLast = false, greenFirst = false,greenInMid = false, picker = false;
    public double id = 0;
    public static PIDCoefficients pidCoefficients, shooterCoef;
    Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        rightshooter = hardwareMap.get(DcMotorEx.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotorEx.class,"leftshooter");
        hood = hardwareMap.get(Servo.class,"hood");
        topturret = hardwareMap.get(DcMotorEx.class,"topturret");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(77,5, Math.toRadians(0)));
        follower.update();
        righttransfer.setDirection(Servo.Direction.REVERSE);
        rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topturret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood.setPosition(hoodup);
        limelight.start();
        greenFirst = false;
        greenInMid = false;
        greenLast = false;
        picker = false;
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (result.isValid()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    id = fr.getFiducialId();
                    telemetry.addData("fudicial", fr.getFiducialId());
                    if (id == 21) {
                        greenFirst = true;
                    } else if (id == 22) {
                        greenInMid = true;
                    } else {
                        greenLast = true;
                    }
                }
            }
        }
        telemetry.update();
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
            if (timer.seconds()<1.2){
                topturret.setPower(pid.calculate(-175,topturret.getCurrentPosition()));
            } else if (timer.seconds()<3) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
            } else if (timer.seconds() < 5) {
                rightshooter.setPower(-1 * shooterpid.calculate(targetvel, leftvel));
                leftshooter.setPower(-1 * shooterpid.calculate(targetvel, leftvel));
                if (greenFirst) {
                    picker = true;
                    midtransfer.setPosition(.7);
                } else {
                    lefttransfer.setPosition(.7);
                }
            } else if (timer.seconds() < 6.2) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                lefttransfer.setPosition(lefttransferservopos);
                midtransfer.setPosition(midtransferservopos);
            } else if (timer.seconds() < 7) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                if (greenInMid){
                    midtransfer.setPosition(.7);
                } else {
                    righttransfer.setPosition(.7);
                }
            } else if (timer.seconds() < 8.2) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                midtransfer.setPosition(midtransferservopos);
                righttransfer.setPosition(righttransferservopos);
            } else if (timer.seconds() < 9) {
                rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                if (greenLast){
                    midtransfer.setPosition(.7);
                } else if (picker) {
                    lefttransfer.setPosition(.7);
                } else {
                    righttransfer.setPosition(.7);
                }
            } else if (timer.seconds() < 10.2) {
                rightshooter.setPower(0);
                midtransfer.setPosition(0);
                leftshooter.setPower(0);
                righttransfer.setPosition(righttransferservopos);
                topturret.setPower(pid.calculate(900,topturret.getCurrentPosition()));
                rightshooter.setPower(0);
                leftshooter.setPower(0);

            } else if  (timer.seconds() < 13.2){
                topturret.setPower(pid.calculate(900,topturret.getCurrentPosition()));
                follower.setTeleOpDrive(0.27, 0, 0);
//                follower.followPath(new Path(new BezierLine(new Pose(77,5, Math.toRadians(0)),new Pose(84,5, Math.toRadians(0)))));
                follower.update();
            } else {
                follower.setTeleOpDrive(0, 0, 0);
                follower.update();
            }

        }
    }
}
