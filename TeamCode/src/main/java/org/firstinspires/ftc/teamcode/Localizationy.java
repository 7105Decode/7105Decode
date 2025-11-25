package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@TeleOp
public class Localizationy extends LinearOpMode {
    DcMotor shooter, frontintake, backintake;
    Limelight3A limelight;
    public static double kp = .01, ki =0, kd=0, target = 0;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        coefficients = new PIDCoefficients(kp,ki,kd);
//        pid = new BasicPID(coefficients);
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(3);
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        frontintake = hardwareMap.get(DcMotor.class,"frontintake");
        backintake = hardwareMap.get(DcMotor.class,"backintake");

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
                }

                if (gamepad1.right_trigger > .3){
                    frontintake.setPower(1);
                } else if (gamepad1.left_trigger > .3){
                    frontintake.setPower(-1);
                } else {
                    frontintake.setPower(0);
                }

                if (gamepad1.dpad_right){
                    shooter.setPower(.33);
                } else if (gamepad1.dpad_left){
                    shooter.setPower(.5);
                } else if (gamepad1.dpad_up){
                    shooter.setPower(.66);
                } else if (gamepad1.a) {
                    shooter.setPower(1);
                } else {
                    shooter.setPower(0);
                }

                if (gamepad2.right_bumper){
                    drive.leftFront.setPower(1);
                } else if (gamepad2.left_bumper) {
                    drive.leftBack.setPower(1);
                } else if (gamepad2.left_trigger>.3) {
                    drive.rightFront.setPower(1); // correct
                } else if (gamepad2.right_trigger > .3) {
                    drive.rightBack.setPower(1);
                } else {
                    drive.leftFront.setPower(0);
                    drive.leftBack.setPower(0);
                    drive.rightBack.setPower(0);
                    drive.rightFront.setPower(0);
                }

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
    }
    public double error(double target,double yaw){
        return target - yaw;
    }
}