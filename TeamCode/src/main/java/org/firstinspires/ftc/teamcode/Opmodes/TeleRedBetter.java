package org.firstinspires.ftc.teamcode.Opmodes;


import static org.firstinspires.ftc.teamcode.Opmodes.TeleBlueBetter.holdpower;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleBlueBetter.leftvel;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleBlueBetter.transferthreshold;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleBlueBetter.uppower;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

@Configurable
@TeleOp
public class TeleRedBetter extends LinearOpMode {
    DcMotorEx rightshooter,leftshooter, frontintake, topturret;
    ElapsedTime timer = new ElapsedTime(), colorSensorResetter = new ElapsedTime()
            ,kickStandTimer = new ElapsedTime();;
    Limelight3A limelight;
    RevColorSensorV3 rightcolorSensor;
    RevColorSensorV3 leftcolorSensor;
    RevColorSensorV3 middlecolorSensor;
    public static boolean hoodUP = false,pidTurretPos = false;
    public static PIDCoefficients pidCoefficients,shooterCoef;
    BasicPID pid,shooterpid;
    Servo righttransfer, midtransfer,lefttransfer, hood, rightled,midled,leftled;
    ShooterStates shooterStates = ShooterStates.OFF;
    TransferStates transferStates = TransferStates.DOWN;
    HoodStates hoodStates = HoodStates.DOWN;
    ParkingStates parkingStates = ParkingStates.DISENGAGE;
    Follower follower;
    CRServo rightkickstand, leftkickstand;
    public static boolean gotRightColor = false, gotMidColor = false, gotLeftColor = false;
    public static double feedforwardlong = 1, feedforwardshort = 0,loopTime,targetvel = 0,ty = 0, shooterkp = 0.024,turretki = 0,turretkd = 0, kp = 0.009,ki = 0,kd = 0,hoodup = .965, hooddown = 0.055,shooterspeed = 0, lefttransferservopos = 0.04, midtransferservopos = .13,righttransferservopos = 0.095, TopTurretPower = .35;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        follower = Constants.createFollower(hardwareMap);

        limelight.pipelineSwitch(1);
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
        colorSensorResetter.reset();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                ty = result.getTy();
                telemetry.addData("ty",ty);
                ty = result.getTy();
                if (ty <= -9){
                    topturret.setPower(-.3);
                } else if (ty > -9 && ty < -.2) {
                    topturret.setPower(-.09);
                } else if (ty >= 9) {
                    topturret.setPower(.3);
                } else if (ty > .2) {
                    topturret.setPower(.09);
                } else{
                    topturret.setPower(0);
                }

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
            double loop = System.nanoTime();
            telemetry.addData("Loop Time ", 1000000000 / (loop - loopTime));
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
            } else if (gotRightColor) {
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
            } else if (gotMidColor){
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
            }else if (gotLeftColor) {
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
                case GOINGUP:
                    rightkickstand.setPower(uppower);
                    leftkickstand.setPower(uppower);
                    if (kickStandTimer.seconds() >= 1.2){
                        parkingStates = ParkingStates.HOLD;
                    }
                    break;
                case HOLD:
                    leftkickstand.setPower(holdpower);
                    rightkickstand.setPower(holdpower);
                    if (gamepad1.right_bumper){
                        kickStandTimer.reset();
                        parkingStates = ParkingStates.GOINGUP;
                    }
                    break;
                case DISENGAGE:
                    rightkickstand.setPower(0);
                    leftkickstand.setPower(0);
                    if (gamepad1.right_bumper){
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
                    rightshooter.setPower(feedforwardlong);
                    leftshooter.setPower(feedforwardlong);
                    if (gamepad1.dpad_down){
                        shooterStates = ShooterStates.OFF;
                    } else if (gamepad1.dpad_left) {
                        targetvel = -1560;
                        shooterkp = 0.012;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
                        shooterStates = ShooterStates.SLOWERSPEED;
                    }
                    break;
                case SLOWERSPEED:
                    rightshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                    leftshooter.setPower(-1*shooterpid.calculate(targetvel,leftvel));
                    if (gamepad1.dpad_up){
                        targetvel = -2280;
                        shooterkp = 0.03;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
                        shooterStates = ShooterStates.MAX;
                    } else if (gamepad1.dpad_down) {
                        shooterStates = ShooterStates.OFF;
                    }
                    break;
                case OFF:
                    rightshooter.setPower(shooterspeed);
                    leftshooter.setPower(shooterspeed);
                    if (gamepad1.dpad_up){
                        targetvel = -2280;
                        shooterkp = 0.03;
                        shooterCoef = new PIDCoefficients(shooterkp,ki,kd);
                        shooterpid = new BasicPID(shooterCoef);
                        shooterStates = ShooterStates.MAX;
                    } else if (gamepad1.dpad_left) {
                        targetvel = -1560;
                        shooterkp = 0.012;
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
                    if (colorSensorResetter.seconds() < .2){
                        gotMidColor = false;
                        gotRightColor = false;
                        gotLeftColor = false;
                    }
                    if (gamepad2.x){
                        timer.reset();
                        transferStates = TransferStates.RIGHTUP;
                    } else if (gamepad2.b) {
                        timer.reset();
                        transferStates = TransferStates.LEFTUP;
                    } else if (gamepad2.a) {
                        timer.reset();
                        transferStates = TransferStates.MIDUP;
                    } else if (gamepad2.left_bumper) {
                        timer.reset();
                        transferStates = TransferStates.FAST;
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
                case FAST:
                     if (timer.seconds() <= .45) {
                    righttransfer.setPosition(.7);
                    midtransfer.setPosition(Transfer.midfurtherback);
                } else if (timer.seconds() <= 1.4) {
                        righttransfer.setPosition(.7);
                        lefttransfer.setPosition(.34);
                    }  else if (timer.seconds() <= 2) {
                         lefttransfer.setPosition(.7);
//                        midtransfer.setPosition(.43);
                         righttransfer.setPosition(righttransferservopos);
                     }  else if (timer.seconds() <= 2.5) {
                         lefttransfer.setPosition(.7);
                         midtransfer.setPosition(.43);
                         righttransfer.setPosition(righttransferservopos);
                     } else if (timer.seconds() <= 2.9) {
                         lefttransfer.setPosition(lefttransferservopos);
                     } else if (timer.seconds() <= 3.5){
                        midtransfer.setPosition(.7);
                        lefttransfer.setPosition(lefttransferservopos);
                    }else {
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
                        gotLeftColor = false;
                        colorSensorResetter.reset();
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
        FAST,
        DOWN
    }
}
