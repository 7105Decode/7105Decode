package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
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
            targetPos = 0, turretKP = .01;
    public static  MotorEx turret;
    public Limelight3A limelight;
    public static LLResult result;
    public String topturretname = "topturret";
    public static boolean startLimelight = false, GPP = false, PGP = false, PPG = false;
    PIDFController pController;
    public static double obeliskID = 0;
    @Override
    public void initialize() {
        turret = new MotorEx(topturretname);
        limelight = OpModeData.hardwareMap.get(Limelight3A.class,"limelight");
        pController = new PIDFController(turretKP);
    }
    @Override
    public void periodic() {
        result = limelight.getLatestResult();
    }
    public void resetEncoder(){
        turret.resetEncoder();
    }
    public static double getTy(){
        return result.getTy();
    }
    public double getTx(){
        return result.getTx();
    }
    public static double getCurrentPosition(){
        return turret.getCurrentPosition();
    }
    public double getPower(){
        return turret.getPower();
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
    public static void aprilTagBangBangTeleop(Gamepad gamepad2){
        if (result.isValid() && getCurrentPosition() > leftSideThreshold && getCurrentPosition() < rightSideThreshold) {
            if (getTy() <= -8.5) {
                turret.setPower(-.3);
            } else if (getTy() > -8.5 && getTy() < .3) {
                turret.setPower(-.09);
            } else if (getTy() >= 9.5) {
                turret.setPower(.3);
            } else if (getTy() > .7) {
                turret.setPower(.09);
            }
        }else if (gamepad2.right_trigger > .3){
            turret.setPower(-.3);
        } else if (gamepad2.left_trigger > .3){
            turret.setPower(.3);
        } else {
            turret.setPower(0);
        }
    }
    public double getError(double reference){
        return reference - getCurrentPosition();
    }
    public Command runPID(double targetPos) {
        return new RunToPosition(turret, // MOTOR TO MOVE
                targetPos, // TARGET POSITION, IN TICKS
                pController, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
}
