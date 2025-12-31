package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tuning.follower;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@Configurable
@TeleOp
public class Tele extends LinearOpMode {
    DcMotor rightshooter,leftshooter, frontintake, topturret;
    ElapsedTime timer = new ElapsedTime();
    Limelight3A limelight;
    RevColorSensorV3 rightcolorSensor;
    RevColorSensorV3 leftcolorSensor;
    RevColorSensorV3 middlecolorSensor;
    public static boolean hoodUP = false,pidTurretPos = false;
    public static PIDCoefficients pidCoefficients;
    BasicPID pid;
    Servo righttransfer, midtransfer,lefttransfer, hood;
    ShooterStates shooterStates = ShooterStates.OFF;
    Follower follower;
    public static double kp = 0.009,ki = 0,kd = 0,hoodup = .965, hooddown = 0.055,shooterspeed = 0, lefttransferservopos = 0.095, midtransferservopos = .13,righttransferservopos = 0.095, TopTurretPower = .35;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        follower = Constants.createFollower(hardwareMap);
        
        limelight.pipelineSwitch(3);
        rightcolorSensor = hardwareMap.get(RevColorSensorV3.class,"rightcolorsensor");
        leftcolorSensor = hardwareMap.get(RevColorSensorV3.class,"leftcolorsensor");
        middlecolorSensor = hardwareMap.get(RevColorSensorV3.class,"middlecolorsensor");
        rightshooter = hardwareMap.get(DcMotor.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotor.class,"leftshooter");
        topturret = hardwareMap.get(DcMotor.class,"topturret");
        frontintake = hardwareMap.get(DcMotor.class,"frontintake");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");
        hood = hardwareMap.get(Servo.class,"hood");


            limelight.start();
            righttransfer.setDirection(Servo.Direction.REVERSE);
            rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            topturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topturret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower.setStartingPose(new Pose(72,72));
        follower.update();

            waitForStart();
        pidCoefficients = new PIDCoefficients(kp,ki,kd);
        pid = new BasicPID(pidCoefficients);
            hoodUP = false;
            pidTurretPos = true;
            shooterStates = ShooterStates.OFF;
            hood.setPosition(hooddown);
            timer.reset();

        follower.startTeleopDrive(true);
        follower.update();

            while (opModeIsActive()) {
//                double heading = Math.toDegrees(pose.heading.toDouble());
//                telemetry.addData("x", pose.position.x);
//                telemetry.addData("y", pose.position.y);
//                telemetry.addData("heading (deg)", heading);
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                follower.update();
                if (gamepad2.right_trigger > .3){
                     topturret.setPower(-.3);
                } else if (gamepad2.left_trigger > .3){
                    topturret.setPower(.3);
                } else if (gamepad2.right_bumper) {
                    topturret.setPower(pid.calculate(-478,topturret.getCurrentPosition()));
                } else if (gamepad2.left_bumper) {
                    topturret.setPower(pid.calculate(478,topturret.getCurrentPosition()));
                } else {
                    topturret.setPower(0);
                }
//                limelight.updateRobotOrientation(heading);
                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
//                    Pose3D botpose = result.getBotpose_MT2();
//                    double tx = result.getTx();
//                    double yaw =botpose.getOrientation().getYaw();
//                    telemetry.addData("tx", tx);
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("botx",botpose.getPosition().x);
//                    telemetry.addData("boty",botpose.getPosition().y);
//                    telemetry.addData("botYaw",yaw);
//                telemetry.addData("pid",pid.calculate(apriltag22.getHeading(), yaw));
                } else {
                    telemetry.addData("Limelight", "No data available");
//                    telemetry.addData("red",rightcolorSensor.red());
//                    telemetry.addData("green",rightcolorSensor.green());
//                    telemetry.addData("blue",rightcolorSensor.blue());
                    telemetry.addData("alpha",rightcolorSensor.rawOptical());
//                    telemetry.addData("red on left",leftcolorSensor.red());
//                    telemetry.addData("green on left",leftcolorSensor.green());
//                    telemetry.addData("blue on left",leftcolorSensor.blue());
                    telemetry.addData("alpha on left",leftcolorSensor.rawOptical());
//                    telemetry.addData("red on middle", middlecolorSensor.red());
//                    telemetry.addData("green on middle",middlecolorSensor.green());
//                    telemetry.addData("blue on middle",middlecolorSensor.blue());
                    telemetry.addData("alpha on middle",middlecolorSensor.rawOptical());
                }

                if (gamepad1.y&& !hoodUP){
                    hood.setPosition(hoodup);
                    hoodUP = true;
                } else if (gamepad1.y && hoodUP){
                    hood.setPosition(hooddown);
                    hoodUP = false;
                }

                if (gamepad1.right_trigger > .3){
                    frontintake.setPower(1);
                } else if (gamepad1.left_trigger > .3){
                    frontintake.setPower(-1);
                } else {
                    frontintake.setPower(0);
                }

                switch (shooterStates) {
                    case MAX:
                        rightshooter.setPower(1);
                        leftshooter.setPower(1);

                        while (gamepad1.dpad_down){
                            shooterStates = ShooterStates.OFF;
                        } while (gamepad1.dpad_left) {
                        shooterStates = ShooterStates.SLOWERSPEED;
                    }
                        break;
                    case SLOWERSPEED:
                        rightshooter.setPower(.7);
                        leftshooter.setPower(.7);
                        while (gamepad1.dpad_up){
                            shooterStates = ShooterStates.MAX;
                        } while (gamepad1.dpad_down) {
                        shooterStates = ShooterStates.OFF;
                    }
                        break;
                    case OFF:
                        rightshooter.setPower(shooterspeed);
                        leftshooter.setPower(shooterspeed);
                        while (gamepad1.dpad_up){
                            shooterStates = ShooterStates.MAX;
                        } while (gamepad1.dpad_left) {
                        shooterStates = ShooterStates.SLOWERSPEED;
                    }
                        break;
                }

                if (gamepad2.b){
                    righttransfer.setPosition(.7);
                } else if (gamepad2.x){
                    lefttransfer.setPosition(.7);
                } else if (gamepad2.a){
                    midtransfer.setPosition(.7);
                }
                else {
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(lefttransferservopos);
                    righttransfer.setPosition(righttransferservopos);
                }


//               This was for testing the drive motors.

//                if (gamepad2.right_bumper){
//                    drive.leftFront.setPower(1);
//                } else if (gamepad2.left_bumper) {
//                    drive.leftBack.setPower(1);
//                } else if (gamepad2.left_trigger>.3) {
//                    drive.rightFront.setPower(1); // correct
//                } else if (gamepad2.right_trigger > .3) {
//                    drive.rightBack.setPower(1);
//                } else {
//                    drive.leftFront.setPower(0);
//                    drive.leftBack.setPower(0);
//                    drive.rightBack.setPower(0);
//                    drive.rightFront.setPower(0);
//                }

                telemetry.update();
            }
    }
    public enum ShooterStates{
        MAX,
        SLOWERSPEED,
        OFF
    }
}
