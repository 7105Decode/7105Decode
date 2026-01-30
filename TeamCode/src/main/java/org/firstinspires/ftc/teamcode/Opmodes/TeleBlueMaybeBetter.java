package org.firstinspires.ftc.teamcode.Opmodes;


import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.kd;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.ki;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.kp;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.turretkd;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.turretki;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.shooterkp;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@Configurable
@TeleOp
public class TeleBlueMaybeBetter extends LinearOpMode {
    DcMotorEx rightshooter,leftshooter, frontintake, topturret;
    ElapsedTime timer = new ElapsedTime();
    Limelight3A limelight;
    RevColorSensorV3 rightcolorSensor;
    RevColorSensorV3 leftcolorSensor;
    RevColorSensorV3 middlecolorSensor;
    public static boolean hoodUP = false,pidTurretPos = false;
    public static PIDCoefficients pidCoefficients,shooterCoef;
    BasicPID pid,shooterpid;
    Servo righttransfer, midtransfer,lefttransfer, hood, rightled,midled,leftled, rightpad, leftpad;
    ShooterStates shooterStates = ShooterStates.OFF;
    TransferStates transferStates = TransferStates.DOWN;
    HoodStates hoodStates = HoodStates.DOWN;
    ParkingStates parkingStates = ParkingStates.DISENGAGE;
    Follower follower;

    public static boolean gotRightColor = false, gotMidColor = false, gotLeftColor = false;
    public static double ty = 0,transferthreshold = 1,leftvel = 0, lpadpos=.274,rpadppos=0.265,targetvel = -2280,hoodup = .965, hooddown = 0.055,shooterspeed = 0, lefttransferservopos = 0.085, midtransferservopos = 0.095,righttransferservopos = 0.11;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        follower = Constants.createFollower(hardwareMap);

        limelight.pipelineSwitch(2);
        rightcolorSensor = hardwareMap.get(RevColorSensorV3.class,"rightcolorsensor");
        leftcolorSensor = hardwareMap.get(RevColorSensorV3.class,"leftcolorsensor");
        middlecolorSensor = hardwareMap.get(RevColorSensorV3.class,"middlecolorsensor");
        rightshooter = hardwareMap.get(DcMotorEx.class,"rightshooter");
        leftshooter = hardwareMap.get(DcMotorEx.class,"leftshooter");
        topturret = hardwareMap.get(DcMotorEx.class,"topturret");
        frontintake = hardwareMap.get(DcMotorEx.class,"frontintake");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");
        hood = hardwareMap.get(Servo.class,"hood");
        rightled = hardwareMap.get(Servo.class,"rightled");
        midled = hardwareMap.get(Servo.class,"midled");
        leftled = hardwareMap.get(Servo.class,"leftled");
        leftpad = hardwareMap.get(Servo.class,"leftpad");
        rightpad = hardwareMap.get(Servo.class,"rightpad");


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
        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
        shooterpid = new BasicPID(shooterCoef);
        hoodUP = false;
        pidTurretPos = true;
        shooterStates = ShooterStates.OFF;
        hood.setPosition(hooddown);
        timer.reset();
        gotLeftColor = false;
        gotRightColor = false;
        gotMidColor = false;

        follower.startTeleopDrive(true);
        follower.update();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                ty = result.getTy();
                    if (ty <= -8.5){
                        topturret.setPower(-.3);
                    } else if (ty > -8.5 && ty < .3) {
                        topturret.setPower(-.09);
                    } else if (ty >= 9.5) {
                        topturret.setPower(.3);
                    } else if (ty > .7) {
                        topturret.setPower(.09);
                    } else{
                    topturret.setPower(0);
                }
                telemetry.addData("ty",ty);
            } else if (gamepad2.right_trigger > .3){
                topturret.setPower(-.3);
            } else if (gamepad2.left_trigger > .3){
                topturret.setPower(.3);
            } else {
                topturret.setPower(0);
            }

            if (gamepad2.right_bumper){
                topturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                topturret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            leftvel = leftshooter.getVelocity();
            telemetry.addData("leftshootervel",leftvel);
            if(rightcolorSensor.rawOptical() >= 300 && !gotRightColor){
                if (rightcolorSensor.red() >= 80){
                    rightled.setPosition(.722);
                    gotRightColor = true;
                } else if (rightcolorSensor.red() >= 60) {
                    rightled.setPosition(.5);
                    gotRightColor = true;
                }
            } else if (rightcolorSensor.rawOptical() > 180) {
                if (rightcolorSensor.red() >= 47){
                    rightled.setPosition(.722);
                    gotRightColor = true;
                } else if (rightcolorSensor.red() >= 26) {
                    rightled.setPosition(.5);
                    gotRightColor = true;
                }
            }else if (rightcolorSensor.rawOptical() > 130) {
                if (rightcolorSensor.red() >= 37){
                    rightled.setPosition(.722);
                    gotRightColor = true;
                } else if (rightcolorSensor.red() >= 26) {
                    rightled.setPosition(.5);
                    gotRightColor = true;
                }
            } else if (!gotRightColor) {
                rightled.setPosition(0);
            }
            if(middlecolorSensor.rawOptical() > 135 && !gotMidColor){
                if (middlecolorSensor.green() >= 90){
                    midled.setPosition(.5);
                    gotMidColor = true;
                } else if (middlecolorSensor.green() >=60) {
                    midled.setPosition(.722);
                    gotMidColor = true;
                }
            } else if (middlecolorSensor.rawOptical() >= 112) {
                if (middlecolorSensor.green() >= 80){
                    midled.setPosition(.5);
                    gotMidColor = true;
                } else if (middlecolorSensor.green() >=50) {
                    midled.setPosition(.722);
                    gotMidColor = true;
                }
            } else if (middlecolorSensor.rawOptical() > 90) {
                if (middlecolorSensor.green() >= 75){
                    midled.setPosition(.5);
                    gotMidColor = true;
                } else if (middlecolorSensor.green() >=50) {
                    midled.setPosition(.722);
                    gotMidColor = true;
                }
            } else if (!gotMidColor){
                midled.setPosition(0);
            }
            if (leftcolorSensor.rawOptical() > 170 && !gotLeftColor){
                if (leftcolorSensor.red() >=69){
                    leftled.setPosition(.722);
                    gotLeftColor = true;
                } else if (leftcolorSensor.red()>= 49) {
                    leftled.setPosition(.5);
                    gotLeftColor = true;
                }
            } else if(leftcolorSensor.rawOptical() > 120){
                if (leftcolorSensor.red() >=52){
                    leftled.setPosition(.722);
                    gotLeftColor = true;
                } else if (leftcolorSensor.red()>= 35) {
                    leftled.setPosition(.5);
                    gotLeftColor = true;
                }
            }else if (!gotLeftColor) {
                leftled.setPosition(0);
            }

            switch (hoodStates){
                case DOWN:
                    hood.setPosition(hooddown);
                    if (gamepad2.dpad_up){
                        hoodStates = HoodStates.UP;
                    }
                    break;
                case UP:
                    hood.setPosition(hoodup);
                    if (gamepad2.dpad_down){
                        hoodStates = HoodStates.DOWN;
                    }
                    break;
            }
            switch (parkingStates){
                case ENGAGEPARK:
                    leftpad.setPosition(.5);
                    rightpad.setPosition(.5);
                    if (gamepad1.right_bumper){
                        parkingStates = ParkingStates.DISENGAGE;
                    }
                    break;
                case DISENGAGE:
                    leftpad.setPosition(lpadpos);
                    rightpad.setPosition(rpadppos);
                    if (gamepad1.right_bumper){
                        parkingStates = ParkingStates.ENGAGEPARK;
                    }
                    break;
            }

            if (gamepad1.left_trigger > .4){
                frontintake.setPower(-1);
            } else if (gamepad1.right_trigger > .4) {
                frontintake.setPower(1);
            } else {
                frontintake.setPower(0);
            }

            switch (shooterStates) {
                case MAX:
                    targetvel = -2280;
                    kp = 0.03;
                    shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                    shooterpid = new BasicPID(shooterCoef);
                    rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                    leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
//                    rightshooter.setPower(1);
//                    leftshooter.setPower(1);
                    if (gamepad1.dpad_down){
                        shooterStates = ShooterStates.OFF;
                    } else if (gamepad1.dpad_left) {
                    shooterStates = ShooterStates.SLOWERSPEED;
                }
                    break;
                case SLOWERSPEED:
                    targetvel = -1560;
                    kp = 0.012;
                    shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                    shooterpid = new BasicPID(shooterCoef);
                    rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                    leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                    if (gamepad1.dpad_up){
                        shooterStates = ShooterStates.MAX;
                    } else if (gamepad1.dpad_down) {
                    shooterStates = ShooterStates.OFF;
                }
                    break;
                case OFF:
                    rightshooter.setPower(shooterspeed);
                    leftshooter.setPower(shooterspeed);
                    if (gamepad1.dpad_up){
                        shooterStates = ShooterStates.MAX;
                    } else if (gamepad1.dpad_left) {
                         shooterStates = ShooterStates.SLOWERSPEED;
                }
                    break;
            }
            switch (transferStates){
                case DOWN:
                    righttransfer.setPosition(righttransferservopos);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(lefttransferservopos);
                    timer.reset();

                    if (gamepad2.x){
                        timer.reset();
                        transferStates = TransferStates.RIGHTUP;
                    } else if (gamepad2.b) {
                        timer.reset();
                        transferStates = TransferStates.LEFTUP;
                    } else if (gamepad2.a) {
                        timer.reset();
                        transferStates = TransferStates.MIDUP;
                    }
                    break;
                case MIDUP:
                    righttransfer.setPosition(righttransferservopos);
                    midtransfer.setPosition(.7);
                    lefttransfer.setPosition(lefttransferservopos);
                    gotMidColor = false;
                    if (timer.seconds() >= transferthreshold) {
                        transferStates = TransferStates.DOWN;
                    }
                    break;
                case RIGHTUP:
                    righttransfer.setPosition(.7);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(lefttransferservopos);
                    gotRightColor = false;
                    if (timer.seconds() >= transferthreshold) {
                        transferStates = TransferStates.DOWN;
                    }
                    break;
                case LEFTUP:
                    righttransfer.setPosition(righttransferservopos);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(.7);
                    gotLeftColor = false;
                    if (timer.seconds() >= transferthreshold) {
                        transferStates = TransferStates.DOWN;
                    }
                    break;
            }
            telemetry.update();
        }
    }
    public enum ShooterStates{
        MAX,
        SLOWERSPEED,
        OFF
    }
    public enum HoodStates{
        UP,
        DOWN
    }
    public enum ParkingStates{
        ENGAGEPARK,
        DISENGAGE
    }
    public enum TransferStates{
        LEFTUP,
        RIGHTUP,
        MIDUP,
        DOWN
    }
}
