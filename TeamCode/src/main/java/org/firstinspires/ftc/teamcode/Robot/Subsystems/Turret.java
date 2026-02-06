package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.teamcode.Robot.MoreConvenientTelemetry;

@Configurable
public class Turret extends Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }
    public static double nintydegrees_right = 750,nintydegrees_left = -750,
            turretforward = 0,rightSideThreshold = 900, leftSideThreshold = -900,
            targetPos = 0;
    public static  MotorEx turret;
    public Limelight3A limelight;
    public static LLResult result;
    public String topturretname = "topturret";
    public boolean startLimelight = false;
    PIDFController pController;

    @Override
    public void initialize() {
        turret = new MotorEx(topturretname);
        limelight = OpModeData.hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
//        pController = new PIDFController();
    }
    @Override
    public void periodic() {
        result = limelight.getLatestResult();
        MoreConvenientTelemetry.addtelem("turretposition", getCurrentPosition());
        MoreConvenientTelemetry.addtelem("limelight_ty",getTy());
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
    public Command runPID() {
        return new RunToPosition(turret, // MOTOR TO MOVE
                targetPos, // TARGET POSITION, IN TICKS
                pController, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
}
