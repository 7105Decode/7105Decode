package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.Robot.MoreConvenientTelemetry;

@Configurable
public class Turret extends Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }
    public static double nintydegrees_right = 750,nintydegrees_left = -750, turretforward = 0;
    public static  MotorEx turret;
    public Limelight3A limelight;
    public static LLResult result;
    public String topturretname = "topturret";
    public boolean startLimelight = false;
    @Override
    public void initialize() {
        turret = new MotorEx(topturretname);
        limelight = OpModeData.hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
    }
    @Override
    public void periodic() {
        result = limelight.getLatestResult();
    }
//    public static void startlimelight(){
//        limelight.start();
//    }
    public void turretTelemetry() {
        MoreConvenientTelemetry.addtelem("turretpower", getPower());
        MoreConvenientTelemetry.addtelem("turretposition",getPosition());
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
    public double getPosition(){
        return turret.getCurrentPosition();
    }
    public double getPower(){
        return turret.getPower();
    }
    public static void aprilTagBangBangTeleop(){
        if (getTy() <= -8.5 && result.isValid()) {
            turret.setPower(-.3);
        } else if (getTy() > -8.5 && getTy() < .3) {
            turret.setPower(-.09);
        } else if (getTy() >= 9.5) {
            turret.setPower(.3);
        } else if (getTy() > .7) {
            turret.setPower(.09);
        } else {
            turret.setPower(0);
        }
    }
    public double getError(double reference){
        return reference - getPosition();
    }
//    public Command runPID() {
//        return new RunToPosition(turret, // MOTOR TO MOVE
//                0.0, // TARGET POSITION, IN TICKS
//                controller, // CONTROLLER TO IMPLEMENT
//                this); // IMPLEMENTED SUBSYSTEM
//    }
}
