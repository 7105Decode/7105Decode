package org.firstinspires.ftc.teamcode.Robot.Subsystems;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.Robot.BetterPanels;

public class Turret extends Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }
    public static double nintydegrees_right = 750,nintydegrees_left = -750, turretforward = 0;
    public MotorEx turret;
//    public Limelight3A limelight;
    public String topturretname = "topturret";
    @Override
    public void initialize() {
        turret = new MotorEx(topturretname);
//        limelight =
    }
    public void turretTelemetry() {
        BetterPanels.addtelem("turretpower", getPower());
        BetterPanels.addtelem("turretposition",getPosition());
    }

    public void resetEncoder(){
        turret.resetEncoder();
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
