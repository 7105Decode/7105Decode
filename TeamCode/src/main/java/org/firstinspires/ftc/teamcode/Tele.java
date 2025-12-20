package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Tuning.draw;
import static org.firstinspires.ftc.teamcode.Tuning.follower;
import static org.firstinspires.ftc.teamcode.Tuning.telemetryM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;

@Config
public class Tele extends OpMode {
    DcMotor shooter, frontintake, backintake, topturret;
    Limelight3A limelight;
    Servo turretservo, transfer;
    public static Follower follower;
    public static double kp = .01, ki =0, kd=0, target = 0, shooterspeed = 0, transferservopos = .13, TopTurretPower = .5;


    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void init () {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(3);
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        topturret = hardwareMap.get(DcMotor.class, "topturret");
        turretservo = hardwareMap.get(Servo.class, "turretservo");
        frontintake = hardwareMap.get(DcMotor.class, "frontintake");
        backintake = hardwareMap.get(DcMotor.class, "backintake");
        transfer = hardwareMap.get(Servo.class, "transfer");

        limelight.start();
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop () {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        double heading = Math.toDegrees(follower.getHeading());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + heading);
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);

        draw();

        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            double tx = result.getTx();
            double yaw = botpose.getOrientation().getYaw();
            telemetry.addData("tx", tx);
            telemetry.addData("ty", result.getTy());
            telemetry.addData("botx", botpose.getPosition().x);
            telemetry.addData("boty", botpose.getPosition().y);
            telemetry.addData("botYaw", yaw);
//               telemetry.addData("pid",pid.calculate(apriltag22.getHeading(), yaw));
        } else {
            telemetry.addData("Limelight", "No data available");
        }


        if (gamepad2.left_trigger > .3) {
            topturret.setPower(-TopTurretPower);
        } else if (gamepad2.right_trigger > .3) {
            topturret.setPower(TopTurretPower);
        } else {
            topturret.setPower(0);
        }
        if (gamepad2.a) {
            transfer.setPosition(.7);
        } else {
            transfer.setPosition(transferservopos);
        }
        if (gamepad1.right_trigger > .3) {
            frontintake.setPower(1);
        } else if (gamepad1.left_trigger > .3) {
            frontintake.setPower(-1);
        } else {
            frontintake.setPower(0);
        }
        if (gamepad1.dpad_right) {
            shooter.setPower(.33);
        } else if (gamepad1.dpad_left) {
            shooter.setPower(.5);
        } else if (gamepad1.dpad_up) {
            shooter.setPower(.66);
        } else if (gamepad1.a) {
            shooter.setPower(1);
        } else {
            shooter.setPower(shooterspeed);
        }
    }
}