package org.firstinspires.ftc.teamcode.Robot.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

public class Turret extends Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }
    public static double nintydegrees_right = 750,nintydegrees_left = -750, turretforward = 0;
    public MotorEx turret;
    public String name = "turret";
    @Override
    public void initialize() {
        turret = new MotorEx(name);
    }
    public void turretTelemetry() {
        BetterDashboard.addtelem("turretpower", getPower());
        BetterDashboard.addtelem("turretposition",getPosition());
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
