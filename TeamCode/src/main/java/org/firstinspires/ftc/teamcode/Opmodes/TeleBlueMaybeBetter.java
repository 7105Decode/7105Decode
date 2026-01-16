package org.firstinspires.ftc.teamcode.Opmodes;


import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.turretkd;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.turretki;
import static org.firstinspires.ftc.teamcode.Opmodes.TeleRedMaybeBetter.turretkp;

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
    public static PIDCoefficients pidCoefficients,shooterCoef,turretpidCo;
    BasicPID pid,shooterpid,turretpid;
    Servo righttransfer, midtransfer,lefttransfer, hood, rightled,midled,leftled, rightpad, leftpad;
    ShooterStates shooterStates = ShooterStates.OFF;
    TransferStates transferStates = TransferStates.DOWN;
    IntakeStates intakeStates = IntakeStates.OFF;
    HoodStates hoodStates = HoodStates.DOWN;
    ParkingStates parkingStates = ParkingStates.DISENGAGE;
    Follower follower;
    public static double transferthreshold = 1.2,leftvel = 0, lpadpos=.274,rpadppos=0.265,targetvel = 2280, error = 0,tx = 0,kp = 0.009,ki = 0,kd = 0,hoodup = .965, hooddown = 0.055,shooterspeed = 0, lefttransferservopos = 0.04, midtransferservopos = .13,righttransferservopos = 0.095, TopTurretPower = .35;
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
        shooterCoef = new PIDCoefficients(kp,ki,kd);
        shooterpid = new BasicPID(shooterCoef);
        turretpidCo = new PIDCoefficients(turretkp,turretki,turretkd);
        turretpid = new BasicPID(turretpidCo);
        hoodUP = false;
        pidTurretPos = true;
        shooterStates = ShooterStates.OFF;
        hood.setPosition(hooddown);
        timer.reset();

        follower.startTeleopDrive(true);
        follower.update();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                tx = result.getTx();
                error = 2.5 - tx;
                if (Math.abs(error) > .2 && topturret.getCurrentPosition() < 900 && topturret.getCurrentPosition() > -900){
//                    topturret.setPower(turretpid.calculate(0,tx));
                    topturret.setPower(turretpid.calculate(0,tx));
                } else{
                    topturret.setPower(0);
                }
            } else if (gamepad2.right_trigger > .3){
                topturret.setPower(-.3);
            } else if (gamepad2.left_trigger > .3){
                topturret.setPower(.3);
            } else if (gamepad2.right_bumper) {
                topturret.setPower(pid.calculate(-484,topturret.getCurrentPosition()));
            } else if (gamepad2.left_bumper) {
                topturret.setPower(pid.calculate(484,topturret.getCurrentPosition()));
            } else {
                topturret.setPower(0);
            }

            leftvel = leftshooter.getVelocity();
            telemetry.addData("leftshootervel",leftvel);
            telemetry.addData("leftshootererror",targetvel - leftvel);
//                telemetry.addData("Limelight", "No data available");
                    telemetry.addData("red",rightcolorSensor.red());
//                    telemetry.addData("green",rightcolorSensor.green());
//                    telemetry.addData("blue",rightcolorSensor.blue());
//                    telemetry.addData("alpha",rightcolorSensor.rawOptical());
                    telemetry.addData("red on left",leftcolorSensor.red());
//                    telemetry.addData("green on left",leftcolorSensor.green());
//                    telemetry.addData("blue on left",leftcolorSensor.blue());
//                    telemetry.addData("alpha on left",leftcolorSensor.rawOptical());
//                    telemetry.addData("red on middle", middlecolorSensor.red());
                    telemetry.addData("green on middle",middlecolorSensor.green());
//                    telemetry.addData("blue on middle",middlecolorSensor.blue());
//                    telemetry.addData("alpha on middle",middlecolorSensor.rawOptical());
//            }
            telemetry.addData("turret",topturret.getCurrentPosition()   );
            if(rightcolorSensor.rawOptical() >= 300){
                if (rightcolorSensor.red() >= 80){
                    rightled.setPosition(.722);
                } else if (rightcolorSensor.red() >= 60) {
                    rightled.setPosition(.5);
                }
            } else if (rightcolorSensor.rawOptical() > 180) {
                if (rightcolorSensor.red() >= 47){
                    rightled.setPosition(.722);
                } else if (rightcolorSensor.red() >= 26) {
                    rightled.setPosition(.5);
                }
            }else if (rightcolorSensor.rawOptical() > 130) {
                if (rightcolorSensor.red() >= 37){
                    rightled.setPosition(.722);
                } else if (rightcolorSensor.red() >= 26) {
                    rightled.setPosition(.5);
                }
            } else {
                rightled.setPosition(0);
            }
            if(middlecolorSensor.rawOptical() > 135){
                if (middlecolorSensor.green() >= 90){
                    midled.setPosition(.5);
                } else if (middlecolorSensor.green() >=60) {
                    midled.setPosition(.722);
                }
            } else if (middlecolorSensor.rawOptical() >= 112) {
                if (middlecolorSensor.green() >= 80){
                    midled.setPosition(.5);
                } else if (middlecolorSensor.green() >=50) {
                    midled.setPosition(.722);
                }
            } else if (middlecolorSensor.rawOptical() > 90) {
                if (middlecolorSensor.green() >= 75){
                    midled.setPosition(.5);
                } else if (middlecolorSensor.green() >=50) {
                    midled.setPosition(.722);
                }
            } else {
                midled.setPosition(0);
            }
            if (leftcolorSensor.rawOptical() > 170){
                if (leftcolorSensor.red() >=69){
                    leftled.setPosition(.722);
                } else if (leftcolorSensor.red()>= 49) {
                    leftled.setPosition(.5);
                }
            } else if(leftcolorSensor.rawOptical() > 120){
                if (leftcolorSensor.red() >=52){
                    leftled.setPosition(.722);
                } else if (leftcolorSensor.red()>= 35) {
                    leftled.setPosition(.5);
                }
            }else {
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

            switch (intakeStates){
                case OFF:
                    frontintake.setPower(0);
                    if (gamepad1.right_trigger > .4){
                        intakeStates = IntakeStates.INTAKE;
                    } else if (gamepad1.left_trigger > .4) {
                        intakeStates = IntakeStates.OUT;
                    }
                    break;
                case INTAKE:
                    frontintake.setPower(1);
                    if (gamepad1.right_trigger > .4){
                        intakeStates = IntakeStates.OFF;
                    } else if (gamepad1.left_trigger > .4) {
                        intakeStates = IntakeStates.OUT;
                    }
                    break;
                case OUT:
                    frontintake.setPower(-1);
                    if (gamepad1.right_trigger > .4){
                        intakeStates = IntakeStates.INTAKE;
                    } else if (gamepad1.left_trigger > .4) {
                        intakeStates = IntakeStates.OFF;
                    }
                    break;
            }

            switch (shooterStates) {
                case MAX:
                    rightshooter.setPower(shooterpid.calculate(targetvel,leftshooter.getVelocity()));
                    leftshooter.setPower(shooterpid.calculate(targetvel,leftshooter.getVelocity()));
//                    rightshooter.setPower(1);
//                    leftshooter.setPower(1);
                    if (gamepad1.dpad_down){
                        shooterStates = ShooterStates.OFF;
                    } else if (gamepad1.dpad_left) {
                    shooterStates = ShooterStates.SLOWERSPEED;
                }
                    break;
                case SLOWERSPEED:
                    rightshooter.setPower(.96);
                    leftshooter.setPower(.96);
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
                        transferStates = TransferStates.RIGHTUP;
                    } else if (gamepad2.b) {
                        transferStates = TransferStates.LEFTUP;
                    } else if (gamepad2.a) {
                        transferStates = TransferStates.MIDUP;
                    }
                    break;
                case MIDUP:
                    righttransfer.setPosition(righttransferservopos);
                    midtransfer.setPosition(.7);
                    lefttransfer.setPosition(lefttransferservopos);
                    if (timer.seconds() >= transferthreshold) {
                        transferStates = TransferStates.DOWN;
                    } else if (gamepad2.x){
                        transferStates = TransferStates.RIGHTUP;
                    } else if (gamepad2.b) {
                        transferStates = TransferStates.LEFTUP;
                    }
                    break;
                case RIGHTUP:
                    righttransfer.setPosition(.7);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(lefttransferservopos);
                    if (timer.seconds() >= transferthreshold) {
                        transferStates = TransferStates.DOWN;
                    } else if (gamepad2.a){
                        transferStates = TransferStates.MIDUP;
                    } else if (gamepad2.b) {
                        transferStates = TransferStates.LEFTUP;
                    }
                    break;
                case LEFTUP:
                    righttransfer.setPosition(righttransferservopos);
                    midtransfer.setPosition(midtransferservopos);
                    lefttransfer.setPosition(.7);
                    if (timer.seconds() >= transferthreshold) {
                        transferStates = TransferStates.DOWN
                        ;
                    } else if (gamepad2.x){
                        transferStates = TransferStates.RIGHTUP;
                    } else if (gamepad2.a) {
                        transferStates = TransferStates.MIDUP;
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
    public enum IntakeStates{
        INTAKE,
        OUT,
        OFF
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
