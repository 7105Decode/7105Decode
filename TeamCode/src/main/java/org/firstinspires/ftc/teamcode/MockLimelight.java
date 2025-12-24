package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//@Config
@TeleOp
public class MockLimelight extends LinearOpMode {
    Limelight3A limelight;

    CRServo servo;
    public static double bigThreshold = 14, smallThreshold = 7, servoMaxPower = 0.07, servoLessPower = -.005;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class,"crservo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("heading (deg)", 0);

//            limelight.updateRobotOrientation(0);
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double tx = result.getTx();
                double yaw = botpose.getOrientation().getYaw();
                telemetry.addData("tx", tx);
                telemetry.addData("ty", result.getTy());
                telemetry.addData("botx", botpose.getPosition().x);
                telemetry.addData("boty", botpose.getPosition().y);
                telemetry.addData("botYaw", yaw);
                findXMiddle(servo,tx);
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();

//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    public void findXMiddle(CRServo crServo, double x){
        if (x > .2){
            crServo.setPower(servoMaxPower);
            telemetry.addLine("firstloop");
        } else if (x < .2){
            crServo.setPower(-servoMaxPower);
            telemetry.addLine("secondloop");
        } else {
            crServo.setPower(0);
        }
    }
}
