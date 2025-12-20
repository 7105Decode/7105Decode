package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;

@Config
public class LocalizationTest extends LinearOpMode {
    DcMotor shooter, frontintake, backintake, topturret;
    Limelight3A limelight;
    RevColorSensorV3 rightcolorSensor;
    Servo righttransfer, midtransfer,lefttransfer;
    public static double kp = .01, ki =0, kd=0, target = 0, shooterspeed = 0, lefttransferservopos = 0.1, midtransferservopos = .78,righttransferservopos = .13, TopTurretPower = .5;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        coefficients = new PIDCoefficients(kp,ki,kd);
//        pid = new BasicPID(coefficients);
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(3);
        rightcolorSensor = hardwareMap.get(RevColorSensorV3.class,"rightcolorsensor");
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        topturret = hardwareMap.get(DcMotor.class,"topturret");
        frontintake = hardwareMap.get(DcMotor.class,"frontintake");
        backintake = hardwareMap.get(DcMotor.class,"backintake");
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");

        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
            limelight.start();
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            waitForStart();

            while (opModeIsActive()) {
                drive.updatePoseEstimate();

                Pose2d pose = drive.pose;
                double heading = Math.toDegrees(pose.heading.toDouble());
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", heading);
                telemetry.update();

                limelight.updateRobotOrientation(heading);
                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();
                    double tx = result.getTx();
                    double yaw =botpose.getOrientation().getYaw();
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("botx",botpose.getPosition().x);
                    telemetry.addData("boty",botpose.getPosition().y);
                    telemetry.addData("botYaw",yaw);
//                telemetry.addData("pid",pid.calculate(apriltag22.getHeading(), yaw));
                } else {
                    telemetry.addData("Limelight", "No data available");
                    telemetry.addData("red",rightcolorSensor.red());
                    telemetry.addData("green",rightcolorSensor.green());
                    telemetry.addData("blue",rightcolorSensor.blue());
                    telemetry.addData("alpha",rightcolorSensor.rawOptical());
                }

                if (gamepad2.left_trigger > .3){
                    topturret.setPower(-TopTurretPower);
                }else if (gamepad2.right_trigger > .3){
                    topturret.setPower(TopTurretPower);
                }else {
                    topturret.setPower(0);
                }

                if (gamepad2.b){
                    righttransfer.setPosition(.6);
                } else {
                    righttransfer.setPosition(righttransferservopos);
                }

                if (gamepad2.x){
                    lefttransfer.setPosition(.35);
                } else {
                    lefttransfer.setPosition(lefttransferservopos);
                }

                if (gamepad2.a){
                    midtransfer.setPosition(.25);
                } else {
                    midtransfer.setPosition(midtransferservopos);
                }

                if (gamepad1.right_trigger > .3){
                    frontintake.setPower(1);
                } else if (gamepad1.left_trigger > .3){
                    frontintake.setPower(-1);
                } else {
                    frontintake.setPower(0);
                }

               if (gamepad1.dpad_left){
                    shooter.setPower(.5);
                } else if (gamepad1.dpad_up){
                    shooter.setPower(.6);
                } else {
                    shooter.setPower(shooterspeed);
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

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}