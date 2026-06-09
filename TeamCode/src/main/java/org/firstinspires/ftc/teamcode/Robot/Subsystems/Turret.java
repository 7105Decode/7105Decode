package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class Turret extends Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }
    public static double nintydegrees_right = 750,nintydegrees_left = -750,
            turretforward = 0,rightSideThreshold = 900, leftSideThreshold = -900,
            targetPos = 0, turretKP = 0.009, driverPower = .3, limelightHighPower = .3, limelightLowPower = .12,
            kp = 0.03, redSideOffsetAuto = -3.3,redSideOffset = -2, blueOffset = 1;
    public MotorEx turret;
    public Limelight3A limelight;
    public LLResult result;

    public String topturretname = "topturret";
    public static boolean doneTrackingAprilTagAuto= false, turnLimelightOn = false,GPP = false, PGP = false, PPG = false;
    public PIDFController pController;
    public static double obeliskID = 0;
    @Override
    public void initialize() {
        turret = new MotorEx(topturretname);
        turret.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight = OpModeData.hardwareMap.get(Limelight3A.class,"limelight");
        pController = new PIDFController(turretKP);
        turnLimelightOn = false;
        doneTrackingAprilTagAuto = false;
    }
    public void resetEncoder(){
        turret.resetEncoder();
    }
    public double getTy(){
        return result.getTy();
    }
    public double getCurrentPosition(){
        return turret.getCurrentPosition();
    }
    public void readObelisk(Telemetry telemetry){
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                obeliskID = fr.getFiducialId();
                telemetry.addData("fudicial", obeliskID);
                if (obeliskID == 21 && result.isValid()) {
                    GPP = true;
                } else if (obeliskID == 22 && result.isValid()) {
                    PGP = true;
                } else {
                    PPG = true;
                }
        }
    }
    public void aprilTagBangBang_Teleop(Gamepad gamepad2){
        if (result.isValid() && getCurrentPosition() > leftSideThreshold && getCurrentPosition() < rightSideThreshold) {
            if (getTy() <= -8.5) {
                turret.setPower(-limelightHighPower);
            } else if (getTy() > -8.5 && getTy() < .3) {
                turret.setPower(-limelightLowPower);
            } else if (getTy() >= 9.5) {
                turret.setPower(limelightHighPower);
            } else if (getTy() > .7) {
                turret.setPower(limelightLowPower);
            }
        }else if (gamepad2.right_trigger > .3){
            turret.setPower(-driverPower);
        } else if (gamepad2.left_trigger > .3){
            turret.setPower(driverPower);
        } else {
            turret.setPower(0);
        }
    }
    public void fishingForAprilTag_RedAuto(boolean turnRight, double power, double offset){
        if (result.isValid()) {
            turret.setPower((offset - getTy())* -kp);
            if ((offset - getTy()) < (offset + 1) && (offset - getTy()) > (offset - 2)){
                doneTrackingAprilTagAuto = true;
            }
        }else if (turnRight){
            Turret.INSTANCE.turret.setPower(power);
        } else {
            Turret.INSTANCE.turret.setPower(-power);
        }
    }
    public void fishingForAprilTag_BlueAuto(boolean turnRight, double power, double offset){
        if (result.isValid()) {
            turret.setPower((offset - getTy())* -kp);
            if ((offset - getTy()) < (offset + 2) && (offset - getTy()) > (offset - 1)){
                doneTrackingAprilTagAuto = true;
            }
        }else if (turnRight){
            Turret.INSTANCE.turret.setPower(power);
        } else {
            Turret.INSTANCE.turret.setPower(-power);
        }
    }
    public void aprilTagBangBang_Auto(){
        if (result.isValid()) {
            if (getTy() <= -8.5) {
                doneTrackingAprilTagAuto = false;
                turret.setPower(-.3);
            } else if (getTy() > -8.5 && getTy() < .3) {
                doneTrackingAprilTagAuto = false;
                turret.setPower(-.09);
            } else if (getTy() >= 9.5) {
                doneTrackingAprilTagAuto = false;
                turret.setPower(.3);
            } else if (getTy() > .7) {
                doneTrackingAprilTagAuto = false;
                turret.setPower(.09);
            } else {
                doneTrackingAprilTagAuto = true;
            }
        } else {
            turret.setPower(0);
        }
    }
    public double getError(double reference){
        return reference - getCurrentPosition();
    }
}
