package org.firstinspires.ftc.teamcode.Opmodes;


import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.feedforwardlong;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.feedforwardshort;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.kd;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.ki;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.kp;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedBetter.shooterkp;

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@Configurable
@TeleOp
public class TeleBlueBetter extends LinearOpMode {
    DcMotorEx rightshooter,leftshooter, frontintake, topturret;
    ElapsedTime timer = new ElapsedTime(), colorSensorResetter = new ElapsedTime()
            ,kickStandTimer = new ElapsedTime();
    Limelight3A limelight;
    RevColorSensorV3 rightcolorSensor;
    RevColorSensorV3 leftcolorSensor;
    RevColorSensorV3 middlecolorSensor;
    public static boolean hoodUP = false;
    public static PIDCoefficients pidCoefficients,shooterCoef;
    BasicPID pid,shooterpid;
    Servo righttransfer, midtransfer,lefttransfer, hood, rightled,midled,leftled;
    ShooterStates shooterStates = ShooterStates.OFF;
    TransferStates transferStates = TransferStates.DOWN;
    HoodStates hoodStates = HoodStates.DOWN;
    ParkingStates parkingStates = ParkingStates.DISENGAGE;
    Follower follower;

    CRServo rightkickstand, leftkickstand;

    public static boolean gotRightColor = false, gotMidColor = false, gotLeftColor = false, useencoder = false;
    public static double loopTime,ty = 0,uppower = 1,holdpower = .08,transferthreshold = 1,leftvel = 0, lpadpos=.274,rpadppos=0.265,targetvel = -2280,hoodup = .965, hooddown = 0.055,shooterspeed = 0, lefttransferservopos = 0.085, midtransferservopos = 0.095,righttransferservopos = 0.115;
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
        rightkickstand = hardwareMap.get(CRServo.class,"rightkickstand");
        leftkickstand = hardwareMap.get(CRServo.class,"leftkickstand");


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

            rightkickstand.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        pidCoefficients = new PIDCoefficients(kp,ki,kd);
        pid = new BasicPID(pidCoefficients);
        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
        shooterpid = new BasicPID(shooterCoef);
        hoodUP = false;
        shooterStates = ShooterStates.OFF;
        hood.setPosition(hooddown);
        timer.reset();
        gotLeftColor = false;
        gotRightColor = false;
        gotMidColor = false;
        useencoder = true;

        follower.startTeleopDrive(true);
        follower.update();
        colorSensorResetter.reset();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            LLResult result = limelight.getLatestResult();
            if (gamepad2.right_trigger > .3){
                topturret.setPower(-.4);
            } else if (gamepad2.left_trigger > .3){
                topturret.setPower(.4);
            } else if (useencoder){
                topturret.setPower(pid.calculate(-535, topturret.getCurrentPosition()));
            }else if (result.isValid()) {
                ty = result.getTy();
//                535
                if (ty <= -8.5){
                    topturret.setPower(-.35);
                } else if (ty > -8.5 && ty < .3) {
                    topturret.setPower(-.11);
                } else if (ty >= 9.5) {
                    topturret.setPower(.35);
                } else if (ty > .7) {
                    topturret.setPower(.11);
                } else{
                    topturret.setPower(0);
                }
                telemetry.addData("ty",ty);
            } else {
                topturret.setPower(0);
            }

            if (gamepad2.dpad_right && useencoder){
                useencoder=false;
            } else if (gamepad2.dpad_right) {
                useencoder=true;
            }

            if (gamepad2.right_bumper){
                topturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                topturret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            leftvel = leftshooter.getVelocity();
            double loop = System.nanoTime();
            telemetry.addData("Loop Time ", 1000000000 / (loop - loopTime));
            telemetry.addData("leftshootervel",leftvel);
            telemetry.addData("turret",topturret.getCurrentPosition());
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
                    gotMidColor = true;}
            } else if (middlecolorSensor.rawOptical() >= 112) {
                if (middlecolorSensor.green() >= 80){
                    midled.setPosition(.5);
                    gotMidColor = true;
                } else if (middlecolorSensor.green() >=50) {
                    midled.setPosition(.722);
                    gotMidColor = true;}
            } else if (middlecolorSensor.rawOptical() > 90) {
                if (middlecolorSensor.green() >= 75){
                    midled.setPosition(.5);
                    gotMidColor = true;
                } else if (middlecolorSensor.green() >=50) {
                    midled.setPosition(.722);
                    gotMidColor = true;}
            } else if (!gotMidColor){
                midled.setPosition(0);}
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
                leftled.setPosition(0);}
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
                case GOINGUP:
                    rightkickstand.setPower(uppower);
                    leftkickstand.setPower(uppower);
                    if (kickStandTimer.seconds() >= 1){
                        parkingStates = ParkingStates.HOLD;
                    }
                    break;
                case HOLD:
                    rightkickstand.setPower(holdpower);
                    leftkickstand.setPower(holdpower);
                    if (gamepad1.dpad_down){
                        kickStandTimer.reset();
                        parkingStates = ParkingStates.DISENGAGE;
                    }
                    break;
                case DISENGAGE:
                    rightkickstand.setPower(0);
                    leftkickstand.setPower(0);
                    if (gamepad1.dpad_up){
                        kickStandTimer.reset();
                        parkingStates = ParkingStates.GOINGUP;
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
                    rightshooter.setPower( (-1*shooterpid.calculate(targetvel,leftvel))+feedforwardlong);
                    leftshooter.setPower( (-1*shooterpid.calculate(targetvel,leftvel)) +feedforwardlong);
                    if (gamepad1.a){
                        shooterStates = ShooterStates.OFF;
                    } else if (gamepad1.left_bumper) {
                        targetvel = -1600;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
                    shooterStates = ShooterStates.SLOWERSPEED;
                }   break;
                case SLOWERSPEED:
                    rightshooter.setPower( (-1*shooterpid.calculate(targetvel,leftvel))+ feedforwardshort);
                    leftshooter.setPower( (-1*shooterpid.calculate(targetvel,leftvel)) + feedforwardshort);
                    if (gamepad1.a) {
                        shooterStates = ShooterStates.OFF;
                    }else if (gamepad1.right_bumper){
                        targetvel = -2280;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
                        shooterStates = ShooterStates.MAX;
                    }
                    break;
                case OFF:
                    rightshooter.setPower(shooterspeed);
                    leftshooter.setPower(shooterspeed);
                    if (gamepad1.right_bumper){
                        targetvel = -2280;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
                        shooterStates = ShooterStates.MAX;
                    } else if (gamepad1.left_bumper) {
                        targetvel = -1600;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
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
                    if (timer.seconds() >= transferthreshold) {
                        gotMidColor = false;
                        colorSensorResetter.reset();
                        transferStates = TransferStates.DOWN;
                    }
                    break;
                case RIGHTUP:
                    righttransfer.setPosition(.7);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(lefttransferservopos);
                    if (timer.seconds() >= transferthreshold) {
                        gotRightColor = false;
                        colorSensorResetter.reset();
                        transferStates = TransferStates.DOWN;
                    }
                    break;
                case LEFTUP:
                    righttransfer.setPosition(righttransferservopos);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(.7);
                    if (timer.seconds() >= transferthreshold) {
                        colorSensorResetter.reset();
                        gotLeftColor = false;
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
        GOINGUP,
        HOLD,
        DISENGAGE
    }
    public enum TransferStates{
        LEFTUP,
        RIGHTUP,
        MIDUP,
        DOWN
    }
}
