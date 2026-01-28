package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.Robot.BetterPanels;

@Configurable
public class Turret extends Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }
    public static double nintydegrees_right = 750,nintydegrees_left = -750, turretforward = 0;
    public MotorEx turret;
    public static Limelight3A limelight;
    LLResult result;
    public String topturretname = "topturret";
    public boolean startLimelight = false;
    @Override
    public void initialize() {
//        result = limelight.getLatestResult();
        turret = new MotorEx(topturretname);
        limelight = OpModeData.hardwareMap.get(Limelight3A.class,"limelight");
    }
    @Override
    public void periodic() {
        result = limelight.getLatestResult();
    }
    public static void startlimelight(){
        limelight.start();
    }
    public void turretTelemetry() {
        BetterPanels.addtelem("turretpower", getPower());
        BetterPanels.addtelem("turretposition",getPosition());
    }
    public void resetEncoder(){
        turret.resetEncoder();
    }
    public double getTy(){
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
